/*
 *   Copyright (C) 2015,2016,2017,2018 by Jonathan Naylor G4KLX
 *   Copyright (C) 2015 by Jim Mclaughlin KI6ZUM
 *   Copyright (C) 2016 by Colin Durbridge G4EML
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "Config.h"
#include "Globals.h"
#include "IO.h"

#include "Log.h"

// Generated using [b, a] = butter(1, 0.001) in MATLAB
static q31_t   DC_FILTER[] = {3367972, 0, 3367972, 0, 2140747704, 0}; // {b0, 0, b1, b2, -a1, -a2}
const uint32_t DC_FILTER_STAGES = 1U; // One Biquad stage

// Generated using rcosdesign(0.2, 8, 5, 'sqrt') in MATLAB
static q15_t RRC_0_2_FILTER[] = {401, 104, -340, -731, -847, -553, 112, 909, 1472, 1450, 683, -675, -2144, -3040, -2706, -770, 2667, 6995,
                                 11237, 14331, 15464, 14331, 11237, 6995, 2667, -770, -2706, -3040, -2144, -675, 683, 1450, 1472, 909, 112,
                                 -553, -847, -731, -340, 104, 401, 0};
const uint16_t RRC_0_2_FILTER_LEN = 42U;

// Generated using rcosdesign(0.2, 8, 10, 'sqrt') in MATLAB
static q15_t NXDN_0_2_FILTER[] = {284, 198, 73, -78, -240, -393, -517, -590, -599, -533, -391, -181, 79, 364, 643, 880, 1041, 1097, 1026, 819,
                                  483, 39, -477, -1016, -1516, -1915, -2150, -2164, -1914, -1375, -545, 557, 1886, 3376, 4946, 6502, 7946, 9184,
                                  10134, 10731, 10935, 10731, 10134, 9184, 7946, 6502, 4946, 3376, 1886, 557, -545, -1375, -1914, -2164, -2150,
                                  -1915, -1516, -1016, -477, 39, 483, 819, 1026, 1097, 1041, 880, 643, 364, 79, -181, -391, -533, -599, -590,
                                  -517, -393, -240, -78, 73, 198, 284, 0};
const uint16_t NXDN_0_2_FILTER_LEN = 82U;

static q15_t NXDN_ISINC_FILTER[] = {790, -1085, -1073, -553, 747, 2341, 3156, 2152, -893, -4915, -7834, -7536, -3102, 4441, 12354, 17394, 17394,
                                   12354, 4441, -3102, -7536, -7834, -4915, -893, 2152, 3156, 2341, 747, -553, -1073, -1085, 790};
const uint16_t NXDN_ISINC_FILTER_LEN = 32U;

// Generated using gaussfir(0.5, 4, 5) in MATLAB
static q15_t   GAUSSIAN_0_5_FILTER[] = {8, 104, 760, 3158, 7421, 9866, 7421, 3158, 760, 104, 8, 0};
const uint16_t GAUSSIAN_0_5_FILTER_LEN = 12U;

// One symbol boxcar filter
static q15_t   BOXCAR_FILTER[] = {12000, 12000, 12000, 12000, 12000, 0};
const uint16_t BOXCAR_FILTER_LEN = 6U;

const uint16_t DC_OFFSET = 0U; //2048U; The SDR transmits samples centered on zero

CIO::CIO() :
m_started(false),
m_thread(),
m_rxBuffer(RX_RINGBUFFER_SIZE),
m_txBuffer(TX_RINGBUFFER_SIZE),
m_rssiBuffer(RX_RINGBUFFER_SIZE),
m_dcFilter(),
m_dcState(),
m_rrcFilter(),
m_gaussianFilter(),
m_boxcarFilter(),
m_nxdnFilter(),
m_nxdnISincFilter(),
m_rrcState(),
m_gaussianState(),
m_boxcarState(),
m_nxdnState(),
m_nxdnISincState(),
m_pttInvert(false),
m_rxLevel(128 * 128),
m_cwIdTXLevel(128 * 128),
m_dstarTXLevel(128 * 128),
m_dmrTXLevel(128 * 128),
m_ysfTXLevel(128 * 128),
m_p25TXLevel(128 * 128),
m_nxdnTXLevel(128 * 128),
m_rxDCOffset(DC_OFFSET),
m_txDCOffset(DC_OFFSET),
m_ledCount(0U),
m_ledValue(true),
m_detect(false),
m_adcOverflow(0U),
m_dacOverflow(0U),
m_watchdog(0U),
m_lockout(false),
m_channelNumber(0),
m_txDelayCounterStarted(false)
{
  ::memset(m_rrcState,      0x00U,  70U * sizeof(q15_t));
  ::memset(m_gaussianState, 0x00U,  40U * sizeof(q15_t));
  ::memset(m_boxcarState,   0x00U,  30U * sizeof(q15_t));
  ::memset(m_nxdnState,     0x00U, 110U * sizeof(q15_t));
  ::memset(m_nxdnISincState, 0x00U, 60U * sizeof(q15_t));
  ::memset(m_dcState,       0x00U,   4U * sizeof(q31_t));

  m_dcFilter.numStages = DC_FILTER_STAGES;
  m_dcFilter.pState    = m_dcState;
  m_dcFilter.pCoeffs   = DC_FILTER;
  m_dcFilter.postShift = 0;

  m_rrcFilter.numTaps = RRC_0_2_FILTER_LEN;
  m_rrcFilter.pState  = m_rrcState;
  m_rrcFilter.pCoeffs = RRC_0_2_FILTER;

  m_gaussianFilter.numTaps = GAUSSIAN_0_5_FILTER_LEN;
  m_gaussianFilter.pState  = m_gaussianState;
  m_gaussianFilter.pCoeffs = GAUSSIAN_0_5_FILTER;

  m_boxcarFilter.numTaps = BOXCAR_FILTER_LEN;
  m_boxcarFilter.pState  = m_boxcarState;
  m_boxcarFilter.pCoeffs = BOXCAR_FILTER;
  
  m_nxdnFilter.numTaps = NXDN_0_2_FILTER_LEN;
  m_nxdnFilter.pState  = m_nxdnState;
  m_nxdnFilter.pCoeffs = NXDN_0_2_FILTER;
  
  m_nxdnISincFilter.numTaps = NXDN_ISINC_FILTER_LEN;
  m_nxdnISincFilter.pState  = m_nxdnISincState;
  m_nxdnISincFilter.pCoeffs = NXDN_ISINC_FILTER;

  initInt();
  
  selfTest();
  setCOSInt(false);
  
}

void CIO::setCN(int cn)
{
  DEBUG2("Using SDR channel %d", cn);
  m_channelNumber = cn;
  m_zmqcontext = zmq::context_t(1);
  m_zmqsocket = zmq::socket_t(m_zmqcontext, ZMQ_PUSH);
  m_zmqsocket.setsockopt(ZMQ_SNDHWM, 2);
  m_zmqsocket.bind ("ipc:///tmp/mmdvm-tx" + std::to_string(cn) + ".ipc");
  
  
  m_zmqcontextRX = zmq::context_t(1);
  m_zmqsocketRX = zmq::socket_t(m_zmqcontextRX, ZMQ_PULL);
  m_zmqsocketRX.setsockopt(ZMQ_RCVHWM, 2);
  m_zmqsocketRX.connect ("ipc:///tmp/mmdvm-rx" + std::to_string(cn) + ".ipc");
}

void CIO::selfTest()
{
  bool ledValue = false;

  for (uint8_t i = 0; i < 6; i++) {
    ledValue = !ledValue;

    // We exclude PTT to avoid trigger the transmitter
    setLEDInt(ledValue);
    setCOSInt(ledValue);
#if defined(ARDUINO_MODE_PINS)
    setDStarInt(ledValue);
    setDMRInt(ledValue);
    setYSFInt(ledValue);
    setP25Int(ledValue);
    setNXDNInt(ledValue);
#endif
    delayInt(250);
  }

#if defined(ARDUINO_MODE_PINS)
  setDStarInt(true);
  setDMRInt(false);
  setYSFInt(false);
  setP25Int(false);
  setNXDNInt(false);

  delayInt(250);

  setDStarInt(true);
  setDMRInt(true);
  setYSFInt(false);
  setP25Int(false);
  setNXDNInt(false);

  delayInt(250);

  setDStarInt(true);
  setDMRInt(true);
  setYSFInt(true);
  setP25Int(false);
  setNXDNInt(false);
  
  delayInt(250);

  setDStarInt(true);
  setDMRInt(true);
  setYSFInt(true);
  setP25Int(true);
  setNXDNInt(false);
  
  delayInt(250);

  setDStarInt(true);
  setDMRInt(true);
  setYSFInt(true);
  setP25Int(true);
  setNXDNInt(true);
  
  delayInt(250);

  setDStarInt(true);
  setDMRInt(true);
  setYSFInt(true);
  setP25Int(true);
  setNXDNInt(false);

  delayInt(250);
  
  setDStarInt(true);
  setDMRInt(true);
  setYSFInt(true);
  setP25Int(false);
  setNXDNInt(false);

  delayInt(250);

  setDStarInt(true);
  setDMRInt(true);
  setYSFInt(false);
  setP25Int(false);
  setNXDNInt(false);

  delayInt(250);
  
  setDStarInt(true);
  setDMRInt(false);
  setYSFInt(false);
  setP25Int(false);
  setNXDNInt(false);

  delayInt(250);

  setDStarInt(false);
  setDMRInt(false);
  setYSFInt(false);
  setP25Int(false);
  setNXDNInt(false);
#endif
}

void CIO::start()
{
  if (m_started)
    return;

  startInt();

  m_started = true;

  setMode(STATE_IDLE);
}

void CIO::process()
{
  m_ledCount++;
  if (m_started) {
    // Two seconds timeout
    if (m_watchdog >= 48000U) {
      if (m_modemState == STATE_DSTAR || m_modemState == STATE_DMR || m_modemState == STATE_YSF || m_modemState == STATE_P25 || m_modemState == STATE_NXDN) {
        if (m_modemState == STATE_DMR && m_tx)
          dmrTX.setStart(false);
        m_modemState = STATE_IDLE;
        setMode(STATE_IDLE);
      }

      m_watchdog = 0U;
    }

    if (m_ledCount >= 24000U) {
      m_ledCount = 0U;
      m_ledValue = !m_ledValue;
      setLEDInt(m_ledValue);
    }
  } else {
    if (m_ledCount >= 240000U) {
      m_ledCount = 0U;
      m_ledValue = !m_ledValue;
      setLEDInt(m_ledValue);
    }
    return;
  }

#if defined(USE_COS_AS_LOCKOUT)
  //m_lockout = getCOSInt();
#endif

    ::pthread_mutex_lock(&m_TXlock);
  // Switch off the transmitter if needed
    if (m_txBuffer.getData() == 0U && m_tx)
    {
    m_tx = false;
    }
  /*
  if (m_txBuffer.getData() == 0U && m_tx && !m_txDelayCounterStarted) {
    m_txDelayCounterStarted = true;
    t1 = std::chrono::high_resolution_clock::now();
  }
  if(m_txDelayCounterStarted)
  {
      t2 = std::chrono::high_resolution_clock::now();
      uint64_t time_delta = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count();
      if(time_delta > 1200000000L)
      {
          if (m_txBuffer.getData() == 0U && m_tx)
          {
            m_tx = false;
          }
          m_txDelayCounterStarted = false;
      }
      if(m_txBuffer.getData() > 0U)
          m_txDelayCounterStarted = false;
  }
  */
  ::pthread_mutex_unlock(&m_TXlock);

  ::pthread_mutex_lock(&m_RXlock);
  u_int16_t block_size = m_rxBuffer.getData();
  ::pthread_mutex_unlock(&m_RXlock);
  
  if (block_size >= RX_BLOCK_SIZE) {
    uint16_t num_blocks = block_size / RX_BLOCK_SIZE;
    for(uint16_t block_no = 0;block_no < num_blocks; block_no++)
    {
    q15_t    samples[RX_BLOCK_SIZE];
    uint8_t  control[RX_BLOCK_SIZE];
    uint16_t rssi[RX_BLOCK_SIZE];

    ::pthread_mutex_lock(&m_RXlock);
    for (uint16_t i = 0U; i < RX_BLOCK_SIZE; i++) {
      int16_t sample;
      m_rxBuffer.get(sample, control[i]);
      m_rssiBuffer.get(rssi[i]);

      // Detect ADC overflow
      //if (m_detect && (sample == 0U || sample == 4095U))
      //  m_adcOverflow++;

      q15_t res1 = q15_t(sample) - m_rxDCOffset;
      q31_t res2 = res1 * m_rxLevel;
      samples[i] = q15_t(__SSAT((res2 >> 15), 16));
    }
    ::pthread_mutex_unlock(&m_RXlock);

    //if (m_lockout)
    //  return;


#if defined(USE_DCBLOCKER)
    q31_t q31Samples[RX_BLOCK_SIZE];
    ::arm_q15_to_q31(samples, q31Samples, RX_BLOCK_SIZE);

    q31_t dcValues[RX_BLOCK_SIZE];
    ::arm_biquad_cascade_df1_q31(&m_dcFilter, q31Samples, dcValues, RX_BLOCK_SIZE);

    q31_t dcLevel = 0;
   for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
      dcLevel += dcValues[i];
    dcLevel /= RX_BLOCK_SIZE;

    q15_t offset = q15_t(__SSAT((dcLevel >> 16), 16));;

    q15_t dcSamples[RX_BLOCK_SIZE];
    for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
      dcSamples[i] = samples[i] - offset;
#endif

    if (m_modemState == STATE_IDLE) {
      if (m_dstarEnable) {
        q15_t GMSKVals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_gaussianFilter, dcSamples, GMSKVals, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_gaussianFilter, samples, GMSKVals, RX_BLOCK_SIZE);
#endif
        dstarRX.samples(GMSKVals, rssi, RX_BLOCK_SIZE);
      }

      if (m_p25Enable) {
        q15_t P25Vals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_boxcarFilter, dcSamples, P25Vals, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_boxcarFilter, samples, P25Vals, RX_BLOCK_SIZE);
#endif
        p25RX.samples(P25Vals, rssi, RX_BLOCK_SIZE);
      }

      if (m_nxdnEnable) {
        q15_t NXDNValsTmp[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_nxdnFilter, dcSamples, NXDNValsTmp, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_nxdnFilter, samples, NXDNValsTmp, RX_BLOCK_SIZE);
#endif
        q15_t NXDNVals[RX_BLOCK_SIZE];
        ::arm_fir_fast_q15(&m_nxdnISincFilter, NXDNValsTmp, NXDNVals, RX_BLOCK_SIZE);

        nxdnRX.samples(NXDNVals, rssi, RX_BLOCK_SIZE);
      }

      if (m_dmrEnable || m_ysfEnable) {
        q15_t RRCVals[RX_BLOCK_SIZE];
        ::arm_fir_fast_q15(&m_rrcFilter, samples, RRCVals, RX_BLOCK_SIZE);

        if (m_ysfEnable)
          ysfRX.samples(RRCVals, rssi, RX_BLOCK_SIZE);

        if (m_dmrEnable) {
          if (m_duplex)
            dmrIdleRX.samples(RRCVals, RX_BLOCK_SIZE);
          else
            dmrDMORX.samples(RRCVals, rssi, RX_BLOCK_SIZE);
        }
      }
    } else if (m_modemState == STATE_DSTAR) {
      if (m_dstarEnable) {
        q15_t GMSKVals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_gaussianFilter, dcSamples, GMSKVals, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_gaussianFilter, samples, GMSKVals, RX_BLOCK_SIZE);
#endif
        dstarRX.samples(GMSKVals, rssi, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_DMR) {
      if (m_dmrEnable) {
        q15_t DMRVals[RX_BLOCK_SIZE];
        ::arm_fir_fast_q15(&m_rrcFilter, samples, DMRVals, RX_BLOCK_SIZE);

        if (m_duplex) {
          // If the transmitter isn't on, use the DMR idle RX to detect the wakeup CSBKs
          if (m_tx)
            dmrRX.samples(DMRVals, rssi, control, RX_BLOCK_SIZE);
          else
            dmrIdleRX.samples(DMRVals, RX_BLOCK_SIZE);
        } else {
          dmrDMORX.samples(DMRVals, rssi, RX_BLOCK_SIZE);
        }
      }
    } else if (m_modemState == STATE_YSF) {
      if (m_ysfEnable) {
        q15_t YSFVals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_rrcFilter, dcSamples, YSFVals, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_rrcFilter, samples, YSFVals, RX_BLOCK_SIZE);
#endif
        ysfRX.samples(YSFVals, rssi, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_P25) {
      if (m_p25Enable) {
        q15_t P25Vals[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_boxcarFilter, dcSamples, P25Vals, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_boxcarFilter, samples, P25Vals, RX_BLOCK_SIZE);
#endif
        p25RX.samples(P25Vals, rssi, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_NXDN) {
      if (m_nxdnEnable) {
        q15_t NXDNValsTmp[RX_BLOCK_SIZE];
#if defined(USE_DCBLOCKER)
        ::arm_fir_fast_q15(&m_nxdnFilter, dcSamples, NXDNValsTmp, RX_BLOCK_SIZE);
#else
        ::arm_fir_fast_q15(&m_nxdnFilter, samples, NXDNValsTmp, RX_BLOCK_SIZE);
#endif
        q15_t NXDNVals[RX_BLOCK_SIZE];
        ::arm_fir_fast_q15(&m_nxdnISincFilter, NXDNValsTmp, NXDNVals, RX_BLOCK_SIZE);

        nxdnRX.samples(NXDNVals, rssi, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_DSTARCAL) {
      q15_t GMSKVals[RX_BLOCK_SIZE];
      ::arm_fir_fast_q15(&m_gaussianFilter, samples, GMSKVals, RX_BLOCK_SIZE);

      calDStarRX.samples(GMSKVals, RX_BLOCK_SIZE);
    } else if (m_modemState == STATE_RSSICAL) {
      calRSSI.samples(rssi, RX_BLOCK_SIZE);
    }
  }
  }
}

void CIO::write(MMDVM_STATE mode, q15_t* samples, uint16_t length, const uint8_t* control)
{

  if (!m_started)
    return;

  if (m_lockout)
    return;

  // Switch the transmitter on if needed
  if (!m_tx) {
    m_tx = true;
    m_txDelayCounterStarted = false;
    setPTTInt(m_pttInvert ? false : true);
  }

  q15_t txLevel = 0;
  switch (mode) {
    case STATE_DSTAR:
      txLevel = m_dstarTXLevel;
      break;
    case STATE_DMR:
      txLevel = m_dmrTXLevel;
      break;
    case STATE_YSF:
      txLevel = m_ysfTXLevel;
      break;
    case STATE_P25:
      txLevel = m_p25TXLevel;
      break;
    case STATE_NXDN:
      txLevel = m_nxdnTXLevel;
      break;
    default:
      txLevel = m_cwIdTXLevel;
      break;
  }
    ::pthread_mutex_lock(&m_TXlock);
  for (uint16_t i = 0U; i < length; i++) {
    q31_t res1 = samples[i] * txLevel;
    q15_t res2 = q15_t(__SSAT((res1 >> 15), 16));
    int16_t res3 = int16_t(res2) + m_txDCOffset;
	//DEBUG2("txDCoffset: %d", m_txDCOffset);

    // Detect DAC overflow
    //if (res3 > 4095U)
      //m_dacOverflow++;

    
   if (control == NULL)
     m_txBuffer.put(res3, MARK_NONE);
   else
     m_txBuffer.put(res3, control[i]);
  }
  ::pthread_mutex_unlock(&m_TXlock);
}

uint16_t CIO::getSpace() 
{
    ::pthread_mutex_lock(&m_TXlock);
    u_int16_t space = m_txBuffer.getSpace();
    ::pthread_mutex_unlock(&m_TXlock);
  return space;
}

void CIO::resetTXBuf() 
{
    int16_t sample;
    uint8_t control;
    ::pthread_mutex_lock(&m_TXlock);
    while(m_txBuffer.get(sample, control))
    {
    }
    ::pthread_mutex_unlock(&m_TXlock);
}

void CIO::setDecode(bool dcd)
{
  if (dcd != m_dcd)
    setCOSInt(dcd ? true : false);

  m_dcd = dcd;
}

void CIO::setADCDetection(bool detect)
{
  m_detect = detect;
}

void CIO::setMode(MMDVM_STATE state)
{
    if (state == m_modemState)
        return;
#if defined(ARDUINO_MODE_PINS)
  setDStarInt(m_modemState == STATE_DSTAR);
  setDMRInt(m_modemState   == STATE_DMR);
  setYSFInt(m_modemState   == STATE_YSF);
  setP25Int(m_modemState   == STATE_P25);
  setNXDNInt(m_modemState  == STATE_NXDN);
#endif
  m_modemState = state;
}

void CIO::setParameters(bool rxInvert, bool txInvert, bool pttInvert, uint8_t rxLevel, uint8_t cwIdTXLevel, uint8_t dstarTXLevel, uint8_t dmrTXLevel, uint8_t ysfTXLevel, uint8_t p25TXLevel, uint8_t nxdnTXLevel, int16_t txDCOffset, int16_t rxDCOffset)
{
  m_pttInvert = pttInvert;

  m_rxLevel      = q15_t(rxLevel * 128);
  m_cwIdTXLevel  = q15_t(cwIdTXLevel * 128);
  m_dstarTXLevel = q15_t(dstarTXLevel * 128);
  m_dmrTXLevel   = q15_t(dmrTXLevel * 128);
  m_ysfTXLevel   = q15_t(ysfTXLevel * 128);
  m_p25TXLevel   = q15_t(p25TXLevel * 128);
  m_nxdnTXLevel  = q15_t(nxdnTXLevel * 128);

  m_rxDCOffset   = DC_OFFSET + rxDCOffset;
  m_txDCOffset   = DC_OFFSET + txDCOffset;
  
  if (rxInvert)
    m_rxLevel = -m_rxLevel;
  
  if (txInvert) {
    m_dstarTXLevel = -m_dstarTXLevel;
    m_dmrTXLevel   = -m_dmrTXLevel;
    m_ysfTXLevel   = -m_ysfTXLevel;
    m_p25TXLevel   = -m_p25TXLevel;
    m_nxdnTXLevel  = -m_nxdnTXLevel;
  }
}

void CIO::getOverflow(bool& adcOverflow, bool& dacOverflow)
{
  adcOverflow = m_adcOverflow > 0U;
  dacOverflow = m_dacOverflow > 0U;

  m_adcOverflow = 0U;
  m_dacOverflow = 0U;
}

bool CIO::hasTXOverflow()
{
    ::pthread_mutex_lock(&m_TXlock);
    bool has_overflowed = m_txBuffer.hasOverflowed();
    ::pthread_mutex_unlock(&m_TXlock);
  return has_overflowed;
}

bool CIO::hasRXOverflow()
{
    ::pthread_mutex_lock(&m_RXlock);
    bool has_overflowed = m_rxBuffer.hasOverflowed();
    ::pthread_mutex_unlock(&m_RXlock);
  return has_overflowed;
}

void CIO::resetWatchdog()
{
  m_watchdog = 0U;
}

uint32_t CIO::getWatchdog()
{
  return m_watchdog;
}

bool CIO::hasLockout() const
{
  return m_lockout;
}

