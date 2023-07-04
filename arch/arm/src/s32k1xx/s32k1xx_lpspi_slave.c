/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_lpspi_slave.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * The external functions, s32k1xx_lpspiNselect* and s32k1xx_lpspiNstatus
 * must be provided by board-specific logic.  They are implementations of the
 * select and status methods of the SPI interface defined by struct
 * s32k1xx_lpspi_ops_s (see include/nuttx/spi/spi.h).  All other methods
 * (including s32k1xx_lpspibus_initialize()) are provided by common S32K1XX
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in s32k1xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide s32k1xx_lpspiNselect() and s32k1xx_lpspiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to s32k1xx_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by s32k1xx_lpspibus_initialize() may then be
 *      used to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_lpspislotinitialize(), for example, will bind the SPI driver
 *      to the SPI MMC/SD driver).
 *
 * NOTE*: If CONFIG_S32K1XX_LPSPI_HWPCS is selected, s32k1xx_lpspiNselect()
 *        does NOT need to provided by board-specific logic.  In this case a
 *        generic implementation is used that switches between native
 *        hardware chip select pins.  It is important that all pins are
 *        configured when the SPI bus is initialized.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <endian.h>
#include <semaphore.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
#include "chip.h"

#include "s32k1xx_pin.h"
#include "hardware/s32k1xx_pinmux.h"
#include "hardware/s32k1xx_lpspi.h"
#include "hardware/s32k1xx_pcc.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_lpspi.h"
#include "s32k1xx_lpspi_slave.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_S32K1XX_SPI_REGDEBUG
static bool     s32k1xx_lpspi_checkreg(struct s32k1xx_lpspidev_s *priv, bool wr,
                                       uint32_t value, uint32_t address);
#else
#  define       s32k1xx_lpspi_checkreg(priv,wr,value,address) (false)
#endif

static 
inline uint32_t s32k1xx_lpspi_getreg32(struct s32k1xx_lpspidev_s *priv,
                                uint8_t offset);
static 
inline void     s32k1xx_lpspi_putreg32(struct s32k1xx_lpspidev_s *priv,
                            uint8_t offset, uint32_t value);

static void     s32k1xx_lpspi_modifyreg32(struct s32k1xx_lpspidev_s *priv,
                                          uint8_t offset, uint32_t clrbits,
                                          uint32_t setbits);

#ifdef CONFIG_DEBUG_SPI_INFO
static void     s32k1xx_lpspi_dumpregs(struct s32k1xx_lpspidev_s *priv, const char *msg);
#else
#  define       s32k1xx_lpspi_dumpregs(priv,msg)
#endif

/* Interrupt Handling */

static int      s32k1xx_lpspi_interrupt(int irq, void *context, void *arg);

/* SPI Helpers */

static uint32_t s32k1xx_lpspi_dequeue(struct s32k1xx_lpspidev_s *priv);
static void     s32k1xx_lpspi_setmode(struct s32k1xx_lpspidev_s *priv,
                                      enum spi_slave_mode_e mode);
static void     s32k1xx_lpspi_setbits(struct s32k1xx_lpspidev_s *priv,
                                      int nbits);

/* SPI slave controller methods */

static void     s32k1xx_lpspi_bind(struct spi_slave_ctrlr_s *ctrlr,
                                   struct spi_slave_dev_s *dev,
                                   enum spi_slave_mode_e mode,
                                   int nbits);
static void     s32k1xx_lpspi_unbind(struct spi_slave_ctrlr_s *ctrlr);
static int      s32k1xx_lpspi_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                                      const void *data, size_t len);
static bool     s32k1xx_lpspi_qfull(struct spi_slave_ctrlr_s *ctrlr);
static void     s32k1xx_lpspi_qflush(struct spi_slave_ctrlr_s *ctrlr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI slave controller driver operations */

static const struct spi_slave_ctrlrops_s g_ctrlr_ops =
{
  .bind    = s32k1xx_lpspi_bind,
  .unbind  = s32k1xx_lpspi_unbind,
  .enqueue = s32k1xx_lpspi_enqueue,
  .qfull   = s32k1xx_lpspi_qfull,
  .qflush  = s32k1xx_lpspi_qflush,
};

static struct s32k1xx_lpspidev_s g_spi2_ctrlr =
{
  .ctrlr   =
  {
    .ops   = &g_ctrlr_ops,
  },
  .base    = S32K1XX_LPSPI2_BASE,
  .spilock = NXMUTEX_INITIALIZER,
  .irq     = S32K1XX_IRQ_LPSPI2,
  .nbits   = 32,
  .nss     = true,
  .initialized = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_lpspi_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_SPI_REGDEBUG
static bool s32k1xx_lpspi_checkreg(struct s32k1xx_lpspidev_s *priv, bool wr, 
                                   uint32_t value, uint32_t address)
{
  if (wr      == priv->wrlast &&     /* Same kind of access? */
      value   == priv->valuelast &&  /* Same value? */
      address == priv->addresslast)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          spiinfo("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast      = wr;
      priv->valuelast   = value;
      priv->addresslast = address;
      priv->ntimes      = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpspi_getreg32
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline
uint32_t s32k1xx_lpspi_getreg32(struct s32k1xx_lpspidev_s *priv,
                                uint8_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_putreg32
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline
void s32k1xx_lpspi_putreg32(struct s32k1xx_lpspidev_s *priv,
                            uint8_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_modifyreg32
 *
 * Description:
 *   Clear and set bits in register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   offset  - Register offset
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void s32k1xx_lpspi_modifyreg32(struct s32k1xx_lpspidev_s *priv,
                                      uint8_t offset, uint32_t clrbits,
                                      uint32_t setbits)
{
  modifyreg32(priv->base + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_dumpregs
 *
 * Description:
 *   Dump the contents of all SPI registers
 *
 * Input Parameters:
 *   priv - The SPI controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SPI_INFO
static void s32k1xx_lpspi_dumpregs(struct s32k1xx_lpspidev_s *priv, const char *msg)
{
  spiinfo("%s:\n", msg);
  spiinfo("  CR:%08x   SR:%08x  IER:%08x\n",
          getreg32(priv->base + S32K1XX_LPSPI_CR_OFFSET),
          getreg32(priv->base + S32K1XX_LPSPI_SR_OFFSET),
          getreg32(priv->base + S32K1XX_LPSPI_IER_OFFSET));
  spiinfo("  CFGR0:%08x CFGR1:%08x\n",
          getreg32(priv->base + S32K1XX_LPSPI_CFGR0_OFFSET),
          getreg32(priv->base + S32K1XX_LPSPI_CFGR1_OFFSET));
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpspi_interrupt
 *
 * Description:
 *   Common SPI interrupt handler
 *
 * Input Parameters:
 *   priv - SPI controller CS state
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int s32k1xx_lpspi_interrupt(int irq, void *context, void *arg)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)arg;
  uint32_t sr;
  uint32_t ier;
  uint32_t pending;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);

  /* We loop because the TDRE interrupt will probably immediately follow the
   * RDRF interrupt and we might be able to catch it in this handler
   * execution.
   */

  for (; ; )
    {
      /* Get the current set of pending/enabled interrupts */

      sr      = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET);
      ier     = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_IER_OFFSET);
      pending = sr & ier;

      /* Return from the interrupt handler when all pending interrupts have
       * been processed.
       */

      if (pending == 0)
        {
          return OK;
        }

      /* The SPI waits until NSS goes active before receiving the serial
       * clock from an external master. When NSS falls, the clock is
       * validated and the data is loaded in the SPI_RDR depending on the
       * BITS field configured in the SPI_CSR0.  These bits are processed
       * following a phase and a polarity defined respectively by the NCPHA
       * and CPOL bits in the SPI_CSR0.
       *
       * When all bits are processed, the received data is transferred in
       * the SPI_RDR and the RDRF bit rises. If the SPI_RDR has not been
       * read before new data is received, the Overrun Error Status (OVRES)
       * bit in the SPI_SR is set. As long as this flag is set, data is
       * loaded in the SPI_RDR. The user must read SPI_SR to clear the OVRES
       * bit.
       */

#ifdef CONFIG_DEBUG_SPI_ERROR
      /* Check the RX data overflow condition */

      if ((pending & LPSPI_SR_REF) != 0)
        {
          /* If debug is enabled, report any overrun errors */

          spierr("ERROR: Overrun (OVRES): %08x\n", pending);

          /* OVRES was cleared by the status read. */

          /* cleared REF flag **Just for testing** */

          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_SR_OFFSET,
                                    0, LPSPI_SR_REF);
        }
#endif

      /* Check for the availability of RX data */

      if ((pending & LPSPI_SR_RDF) != 0)
        {
          uint32_t data;

          /* We get no indication of the falling edge of NSS.  But if we are
           * here then it must have fallen.
           */

          if (priv->nss)
            {
              priv->nss = false;
              SPIS_DEV_SELECT(priv->dev, true);
            }

          /* Read the RDR to get the data and to clear the pending RDRF
           * interrupt.
           */

          regval = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_RDR_OFFSET);
          data   = (uint32_t)
            ((regval & LPSPI_RDR_DATA_MASK) >> LPSPI_RDR_DATA_SHIFT);

          /* Enable TEF/TDF interrupts */

          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_IER_OFFSET,
                                    0, LPSPI_IER_TEIE | LPSPI_IER_TDIE);

          /* Report the receipt of data to the SPI device driver */

          SPIS_DEV_RECEIVE(priv->dev, (const uint32_t *)&data,
                           sizeof(data));
        }

      /* When a transfer starts, the data shifted out is the data present
       * in the Shift register. If no data has been written in the SPI_TDR,
       * the last data received is transferred. If no data has been received
       * since the last reset, all bits are transmitted low, as the Shift
       * register resets to 0.
       *
       * When a first data is written in the SPI_TDR, it is transferred
       * immediately in the Shift register and the TDRE flag rises. If new
       * data is written, it remains in the SPI_TDR until a transfer occurs,
       * i.e., NSS falls and there is a valid clock on the SPCK pin. When
       * the transfer occurs, the last data written in the SPI_TDR is
       * transferred in the Shift register and the TDRE flag rises. This
       * enables frequent updates of critical variables with single
       * transfers.
       *
       * Then, new data is loaded in the Shift register from the SPI_TDR. If
       * no character is ready to be transmitted, i.e., no character has been
       * written in the SPI_TDR since the last load from the SPI_TDR to the
       * Shift register, the SPI_TDR is retransmitted. In this case the
       * Underrun Error Status Flag (UNDES) is set in the SPI_SR.
       */

#ifdef CONFIG_DEBUG_SPI_ERROR
      /* Check the TX data underflow condition */

      if ((pending & LPSPI_SR_TEF) != 0)
        {
          /* If debug is enabled, report any overrun errors */

          spierr("ERROR: Underrun (UNDEX): %08x\n", pending);

          /* UNDES was cleared by the status read. */

          /* cleared REF flag **Just for testing** */

          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_SR_OFFSET,
                                    0, LPSPI_SR_TEF);
        }
#endif

      /* Output the next TX data */

      if ((pending & LPSPI_SR_TDF) != 0)
        {
          /* Get the next output value and write it to the TDR
           * The TDRE interrupt is cleared by writing to the from RDR.
           */

          regval = s32k1xx_lpspi_dequeue(priv);
          s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_TDR_OFFSET, regval);
        }

      /* The SPI slave hardware provides only an event when NSS rises
       * which may or many not happen at the end of a transfer.  NSSR was
       * cleared by the status read.
       */

      if ((pending & LPSPI_SR_FCF) != 0)
        {
          /* Disable further TEF/TDF interrupts */

          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_IER_OFFSET,
                                    LPSPI_IER_TEIE | LPSPI_IER_TDIE, 0);

          /* Report the state change to the SPI device driver */

          priv->nss = true;
          SPIS_DEV_SELECT(priv->dev, false);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_lpspi_dequeue
 *
 * Description:
 *   Return the next queued output value.  If nothing is in the output queue,
 *   then return the last value obtained from getdata();
 *
 * Input Parameters:
 *   priv - SPI controller CS state
 *
 * Assumptions:
 *   Called only from the SPI interrupt handler so all interrupts are
 *   disabled.
 *
 ****************************************************************************/

static uint32_t s32k1xx_lpspi_dequeue(struct s32k1xx_lpspidev_s *priv)
{
  uint32_t ret;
  int next;

  /* Is the queue empty? */

  if (priv->tx_head != priv->tx_tail)
    {
      /* No, take the oldest value from the tail of the circular buffer */

      ret = priv->outq[priv->tx_tail];

      /* Update the tail index, handling wraparound */

      next = priv->tx_tail + 1;
      if (next >= CONFIG_SPI_SLAVE_QSIZE)
        {
          next = 0;
        }

      priv->tx_tail = next;

      /* If the queue is empty Disable further TXDR/OVRE interrupts until
       * spi_enqueue() is called or until we received another command.  We
       * do this only for the case where NSS is non-functional (tied to
       * ground) and we need to end transfers in some fashion.
       */

      if (priv->tx_head == next)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_IER_OFFSET,
                                    LPSPI_IER_TEIE | LPSPI_IER_TDIE, 0);
        }
    }
  else
    {
      /* Yes, return the last value we got from the getdata() method */

      ret = priv->outval;

      /* Disable further TXDR/OVRE interrupts until spi_enqueue() is called */

      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_IER_OFFSET,
                                LPSPI_IER_TEIE | LPSPI_IER_TDIE, 0);
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_lpspi_setmode
 *
 * Description:
 *   Set the SPI Slave mode.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   mode  - Requested SPI Slave mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void s32k1xx_lpspi_setmode(struct s32k1xx_lpspidev_s *priv,
                                  enum spi_slave_mode_e mode)
{
  uint32_t setbits;
  uint32_t clrbits;
  uint32_t men;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Disable LPSPI if it is enabled */

      men = getreg32(S32K1XX_LPSPI2_CR) & LPSPI_CR_MEN;
      if (men)
        {
          modifyreg32(S32K1XX_LPSPI2_CR, LPSPI_CR_MEN, 0);
        }

      switch (mode)
        {
        case SPIDEV_MODE0:     /* CPOL=0; CPHA=0 */
          setbits = 0;
          clrbits = LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
          break;

        case SPIDEV_MODE1:     /* CPOL=0; CPHA=1 */
          setbits = LPSPI_TCR_CPHA;
          clrbits = LPSPI_TCR_CPOL;
          break;

        case SPIDEV_MODE2:     /* CPOL=1; CPHA=0 */
          setbits = LPSPI_TCR_CPOL;
          clrbits = LPSPI_TCR_CPHA;
          break;

        case SPIDEV_MODE3:     /* CPOL=1; CPHA=1 */
          setbits = LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
          clrbits = 0;
          break;

        default:
          spierr("Invalid mode: %d\n", mode);
          DEBUGPANIC();
          return;
        }

      modifyreg32(S32K1XX_LPSPI2_TCR, clrbits, setbits);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          modifyreg32(S32K1XX_LPSPI2_CR, 0, LPSPI_CR_MEN);
        }

    }
}

/****************************************************************************
 * Name: s32k1xx_lpspi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   nbits - The number of bits in an SPI word
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void     s32k1xx_lpspi_setbits(struct s32k1xx_lpspidev_s *priv,
                                      int nbits)
{
  uint32_t regval;
  uint32_t men;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      if (nbits < 2 || nbits > 4096)
        {
          return;
        }

      /* Disable LPSPI if it is enabled */

      men = getreg32(S32K1XX_LPSPI2_CR) & LPSPI_CR_MEN;
      if (men)
        {
          modifyreg32(S32K1XX_LPSPI2_CR, LPSPI_CR_MEN, 0);
        }

      regval = getreg32(S32K1XX_LPSPI2_TCR);
      regval &= ~LPSPI_TCR_FRAMESZ_MASK;
      regval |= LPSPI_TCR_FRAMESZ(nbits - 1);

      putreg32(regval, S32K1XX_LPSPI2_TCR);

      /* Save the selection so that subsequent re-configurations will
       * be faster.
       */

      priv->nbits = nbits;    /* nbits has been clobbered... save the signed
                               * value. */

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          modifyreg32(S32K1XX_LPSPI2_CR, 0, LPSPI_CR_MEN);
        }
    }
}

/****************************************************************************
 * Name: s32k1xx_lpspi_bind
 *
 * Description:
 *   Bind the SPI Slave device interface to the SPI Slave controller
 *   interface and configure the SPI interface. Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   dev   - SPI Slave device interface instance
 *   mode  - The SPI mode requested
 *   nbits - The number of bits requests.
 *            If value is greater than 0, then it implies MSB first
 *            If value is less than 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This implementation currently supports only positive "nbits" values,
 *   i.e., it always configures the SPI Slave controller driver as MSB first.
 *
 ****************************************************************************/

static void s32k1xx_lpspi_bind(struct spi_slave_ctrlr_s *ctrlr,
                               struct spi_slave_dev_s *dev,
                               enum spi_slave_mode_e mode,
                               int nbits)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)ctrlr;
  uint32_t regval;
  int ret;
  const void *data;

  spiinfo("dev=%p mode=%d nbits=%d\n", dev, mode, nbits);

  DEBUGASSERT(priv != NULL && priv->dev == NULL && dev != NULL);

  /* Get exclusive access to the SPI device */

  ret = nxmutex_lock(&priv->spilock);
  if (ret < 0)
    {
      /* REVISIT:  No mechanism to report error.  This error should only
       * occur if the calling task was canceled.
       */

      spierr("RROR: nxmutex_lock failed: %d\n", ret);
      return;
    }

  /* Bind the SPI slave device interface instance to the SPI slave
   * controller interface.
   */

  priv->dev = dev;

  /* Call the slaved device's select() and cmddata() methods to indicate
   * the initial state of the chip select and  command/data discretes.
   *
   * NOTE:  Unless we reconfigure the NSS GPIO pin, it may not be possible
   * to read the NSS pin value (I haven't actually tried just reading it).
   * And, since the is no interrupt on the falling edge of NSS, we get no
   * notification when we are selected... not until the arrival of data.
   *
   * REVISIT:  A board-level interface would be required in order to support
   * the Command/Data indication (not yet impklemented).
   */

  SPIS_DEV_SELECT(dev, false);
#warning Missing logic
  SPIS_DEV_CMDDATA(dev, false);

  /* Discard any queued data */

  priv->tx_head  = 0;
  priv->tx_tail  = 0;

  /* Call the slave device's getdata() method to get the value that will
   * be shifted out the SPI clock is detected.
   */

  SPIS_DEV_GETDATA(dev, &data);
  priv->outval = *(const uint32_t *)data;
  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_TDR_OFFSET, priv->outval);

  /* Setup to begin normal SPI operation */

  s32k1xx_lpspi_setmode(priv, mode);
  s32k1xx_lpspi_setbits(priv, nbits);

  /* Clear pending interrupts by reading the SPI Status Register */

  regval = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET);
  UNUSED(regval);

  /* Enable SPI interrupts (already enabled at the NVIC):
   *
   * Data Transfer:
   *   SPI_INT_RDRF - Receive Data Register Full Interrupt
   *   SPI_INT_TDRE - Transmit Data Register Empty Interrupt
   *   SPI_INT_NSSR - NSS Rising Interrupt
   *
   * Transfer Errors (for DEBUG purposes only):
   *   SPI_INT_OVRES - Overrun Error Interrupt
   *   SPI_INT_UNDES - Underrun Error Status Interrupt (slave)
   *
   * Not Used:
   *  SPI_INT_MODF    - Mode Fault Error Interrupt
   *  SPI_INT_TXEMPTY  - Transmission Registers Empty Interrupt
   *
   * TX interrupts (SPI_INT_TDRE and SPI_INT_UNDES) are not enabled until
   * the transfer of data actually starts.
   */

  regval  = (LPSPI_IER_RDIE | LPSPI_IER_FCIE);
#ifdef CONFIG_DEBUG_SPI_ERROR
  regval |= LPSPI_IER_TDIE;
#endif

  s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_IER_OFFSET, 0, regval);

  nxmutex_unlock(&priv->spilock);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_unbind
 *
 * Description:
 *   Un-bind the SPI Slave device interface from the SPI Slave controller
 *   interface. Reset the SPI interface and restore the SPI Slave
 *   controller driver to its initial state.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void s32k1xx_lpspi_unbind(struct spi_slave_ctrlr_s *ctrlr)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)ctrlr;

  DEBUGASSERT(priv != NULL);
  spiinfo("Unbinding %p\n", priv->dev);

  DEBUGASSERT(priv->dev != NULL);

  /* Get exclusive access to the SPI device */

  nxmutex_lock(&priv->spilock);

  /* Disable SPI interrupts (still enabled at the NVIC) */

  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_IER_OFFSET, 0);

  /* Unbind the SPI slave interface */

  priv->dev = NULL;

  /* Disable the SPI peripheral */

  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CR_OFFSET, ~LPSPI_CR_MEN);

  /* Execute a software reset of the SPI (twice) */

  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CR_OFFSET, LPSPI_CR_RST);
  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CR_OFFSET, LPSPI_CR_RST);

  nxmutex_unlock(&priv->spilock);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_enqueue
 *
 * Description:
 *   Enqueue the next value to be shifted out from the interface. This adds
 *   the word to the controller driver for a subsequent transfer but has no
 *   effect on any in-process or currently "committed" transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   data  - Pointer to the command/data mode data to be shifted out.
 *           The data width must be aligned to the nbits parameter which was
 *           previously provided to the bind() method.
 *   len   - Number of units of "nbits" wide to enqueue,
 *           "nbits" being the data width previously provided to the bind()
 *           method.
 *
 * Returned Value:
 *   Number of data items successfully queued, or a negated errno:
 *         - "len" if all the data was successfully queued
 *         - "0..len-1" if queue is full
 *         - "-errno" in any other error
 *
 ****************************************************************************/

static int s32k1xx_lpspi_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                            const void *data,
                            size_t len)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)ctrlr;
  irqstate_t flags;
  int next;
  int ret;

  spiinfo("data=%04x\n", *(const uint32_t *)data);
  DEBUGASSERT(priv != NULL && priv->dev != NULL);

  /* Get exclusive access to the SPI device */

  ret = nxmutex_lock(&priv->spilock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if this word would overflow the circular buffer
   *
   * Interrupts are disabled briefly.
   */

  flags = enter_critical_section();
  next = priv->tx_head + 1;
  if (next >= CONFIG_SPI_SLAVE_QSIZE)
    {
      next = 0;
    }

  if (next == priv->tx_tail)
    {
      ret = -ENOSPC;
    }
  else
    {
      /* Save this new word as the next word to shifted out.  The current
       * word written to the TX data registers is "committed" and will not
       * be overwritten.
       */

      priv->outq[priv->tx_head] = *(const uint32_t *)data;
      priv->tx_head = next;
      ret = OK;

      /* Enable TX interrupts if we have begun the transfer */

      if (!priv->nss)
        {
          /* Enable TXDR/OVRE interrupts */

          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_IER_OFFSET,
                                     0, LPSPI_IER_TEIE | LPSPI_IER_TDIE);
        }
    }

  leave_critical_section(flags);
  nxmutex_unlock(&priv->spilock);
  return ret;
}

/****************************************************************************
 * Name: s32k1xx_lpspi_qfull
 *
 * Description:
 *   Return true if the queue is full or false if there is space to add an
 *   additional word to the queue.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   true if the output queue is full, false otherwise.
 *
 ****************************************************************************/

static bool s32k1xx_lpspi_qfull(struct spi_slave_ctrlr_s *ctrlr)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)ctrlr;
  irqstate_t flags;
  bool bret;
  int ret;
  int next;

  DEBUGASSERT(priv != NULL && priv->dev != NULL);

  /* Get exclusive access to the SPI device */

  ret = nxmutex_lock(&priv->spilock);
  if (ret < 0)
    {
      /* REVISIT:  No mechanism to report error.  This error should only
       * occurr if the calling task was canceled.
       */

      spierr("RROR: nxmutex_lock failed: %d\n", ret);
      return true;
    }

  /* Check if another word would overflow the circular buffer
   *
   * Interrupts are disabled briefly.
   */

  flags = enter_critical_section();
  next = priv->tx_head + 1;
  if (next >= CONFIG_SPI_SLAVE_QSIZE)
    {
      next = 0;
    }

  bret = (next == priv->tx_tail);
  leave_critical_section(flags);
  nxmutex_unlock(&priv->spilock);
  return bret;
}

/****************************************************************************
 * Name: s32k1xx_lpspi_qflush
 *
 * Description:
 *   Discard all saved values in the output queue. On return from this
 *   function the output queue will be empty. Any in-progress or otherwise
 *   "committed" output values may not be flushed.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void s32k1xx_lpspi_qflush(struct spi_slave_ctrlr_s *ctrlr)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)ctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && priv->dev != NULL);

  /* Get exclusive access to the SPI device */

  nxmutex_lock(&priv->spilock);

  /* Mark the buffer empty, momentarily disabling interrupts */

  flags = enter_critical_section();
  priv->tx_head = 0;
  priv->tx_tail = 0;
  leave_critical_section(flags);
  nxmutex_unlock(&priv->spilock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_spislave_ctrlr_initialize
 *
 * Description:
 *   Initialize the selected SPI Slave bus.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple SPI Slave interfaces)
 *
 * Returned Value:
 *   Valid SPI Slave controller structure reference on success;
 *   NULL on failure.
 *
 ****************************************************************************/

struct spi_slave_ctrlr_s *s32k1xx_spi_slave_initialize(int bus)
{
  struct s32k1xx_lpspidev_s *priv;
  irqstate_t flags;

  spiinfo("spi bus: %d\n", bus);

  DEBUGASSERT(bus >= 0 && bus <= 2);

  /* Select SPI2 */

  priv = &g_spi2_ctrlr;

  /* Has the SPI hardware been initialized? */

  if (!priv->initialized)
    {
      flags = enter_critical_section();

      /* Enable peripheral clocking to SPI2 */

      s32k1xx_pclk_enable(LPSPI2_CLK, true);

      /* Configure multiplexed pins as connected on the board. */

      s32k1xx_pinconfig(PIN_LPSPI2_MISO); /* Output */
      s32k1xx_pinconfig(PIN_LPSPI2_MOSI); /* Input */
      s32k1xx_pinconfig(PIN_LPSPI2_SCK);  /* Drives slave */
      s32k1xx_pinconfig(PIN_LPSPI2_PCS);

      /* Disable the SPI peripheral */

      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);

      /* Execute a software reset of the SPI (twice) */

      s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 
                             LPSPI_CR_RST | LPSPI_CR_RTF | LPSPI_CR_RRF);
      s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 
                             LPSPI_CR_RST | LPSPI_CR_RTF | LPSPI_CR_RRF);
      
      s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0);
      leave_critical_section(flags);

      /* Select PCS pin LPSPI_PCS[3] (PIN_LPSPI2_PCS0_4) */

      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_TCR_OFFSET, 0, LPSPI_TCR_PCS_3);

      /* Set water masks */

      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_FCR_OFFSET, 
                                LPSPI_FCR_TXWATER_MASK, LPSPI_FCR_TXWATER(3));

      /* Enable BYSW */

      // s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_TCR_OFFSET, 0, LPSPI_TCR_BYSW);  
      
      /* Enable LPSPI */

      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);

      /* Disable all SPI interrupts at the SPI peripheral */

      s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_IER_OFFSET, 0);

      /* Attach and enable interrupts at the NVIC */

      DEBUGVERIFY(irq_attach(priv->irq, s32k1xx_lpspi_interrupt, priv));
      up_enable_irq(priv->irq);

      s32k1xx_lpspi_dumpregs(priv, "After initialization");


      priv->initialized = true;
    }
      
  /* Set Transmit Command Register */

  s32k1xx_lpspi_setbits(priv, 32);
  s32k1xx_lpspi_setmode(priv, SPIDEV_MODE1);

  return &priv->ctrlr;
}
