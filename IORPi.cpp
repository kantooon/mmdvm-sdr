/*
 *   Copyright (C) 2015,2016,2017,2018 by Jonathan Naylor G4KLX
 *   Copyright (C) 2015 by Jim Mclaughlin KI6ZUM
 *   Copyright (C) 2016 by Colin Durbridge G4EML
 * 
 *   GNU radio integration code written by Adrian Musceac YO8RZZ 2021
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
#include <pthread.h>
#include <vector>

#if defined(RPI)

#include "Log.h"

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <chrono>


const uint16_t DC_OFFSET = 2048U;

unsigned char wavheader[] = {0x52,0x49,0x46,0x46,0xb8,0xc0,0x8f,0x00,0x57,0x41,0x56,0x45,0x66,0x6d,0x74,0x20,0x10,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0xc0,0x5d,0x00,0x00,0x80,0xbb,0x00,0x00,0x02,0x00,0x10,0x00,0x64,0x61,0x74,0x61,0xff,0xff,0xff,0xff};
std::chrono::high_resolution_clock::time_point tm1 = std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point tm2 = std::chrono::high_resolution_clock::now();

void CIO::initInt()
{

//	std::cout << "IO Init" << std::endl;
	DEBUG1("IO Init done! Thread Started!");

}

void CIO::startInt()
{
    
	DEBUG1("IO Int start()");
    if (::pthread_mutex_init(&m_TXlock, NULL) != 0)
    {
        printf("\n Tx mutex init failed\n");
        exit(1);;
    }
    if (::pthread_mutex_init(&m_RXlock, NULL) != 0)
    {
        printf("\n RX mutex init failed\n");
        exit(1);;
    }

    ::pthread_create(&m_thread, NULL, helper, this);
    ::pthread_create(&m_threadRX, NULL, helperRX, this);
    ::pthread_setname_np(m_thread, "mmdvm_tx");
    ::pthread_setname_np(m_threadRX, "mmdvm_rx");
}

void* CIO::helper(void* arg)
{
  CIO* p = (CIO*)arg;

  while (1)
  {
      if(p->m_txBuffer.getData() < 1)
        usleep(20);
    p->interrupt();
  }

  return NULL;
}

void* CIO::helperRX(void* arg)
{
  CIO* p = (CIO*)arg;

  while (1)
  {

    usleep(200);
    p->interruptRX();
  }

  return NULL;
}


void CIO::interrupt()
{

    uint16_t sample = DC_OFFSET;
    uint8_t control = MARK_NONE;
    uint32_t num_items = 720;
    bool wait_for_data = false;
    ::pthread_mutex_lock(&m_TXlock);
    if(m_txBuffer.getData() >= num_items)
    {
        while(m_txBuffer.get(sample, control))
        {

            sample *= 5;		// amplify by 12dB	
            short signed_sample = (short)sample;
            m_samplebuf.push_back(signed_sample);
            m_controlbuf.push_back(control);

            if(m_samplebuf.size() >= num_items)
            {
                int buf_size = sizeof(uint32_t) + num_items * sizeof(uint8_t) + num_items * sizeof(int16_t);
                
                zmq::message_t reply (buf_size);
                memcpy (reply.data (), &num_items, sizeof(uint32_t));
                memcpy ((unsigned char *)reply.data () + sizeof(uint32_t), (unsigned char *)m_controlbuf.data(), num_items * sizeof(uint8_t));
                memcpy ((unsigned char *)reply.data () + sizeof(uint32_t) + num_items * sizeof(uint8_t),
                        (unsigned char *)m_samplebuf.data(), num_items*sizeof(int16_t));
                m_zmqsocket.send (reply, zmq::send_flags::dontwait);
                wait_for_data = true;
                m_samplebuf.erase(m_samplebuf.begin(), m_samplebuf.begin()+num_items);
                m_controlbuf.erase(m_controlbuf.begin(), m_controlbuf.begin()+num_items);
            }
        }
        ::pthread_mutex_unlock(&m_TXlock);
    }
    else
    {
        ::pthread_mutex_unlock(&m_TXlock);
        usleep(20);
    }
       
   if (wait_for_data)
   {
        tm1 = std::chrono::high_resolution_clock::now();
        tm2 = std::chrono::high_resolution_clock::now();
        while(std::chrono::duration_cast<std::chrono::nanoseconds>(tm2-tm1).count() < 29900000L)
        {
            usleep(10);
            tm2 = std::chrono::high_resolution_clock::now();
        }
       wait_for_data = false;
   }
   
   
   
   
   
   
    sample = 2048U;

#if defined(SEND_RSSI_DATA)
    //m_rssiBuffer.put(ADC->ADC_CDR[RSSI_CDR_Chan]);
#else
    //m_rssiBuffer.put(0U);
#endif

    //m_watchdog++;
	
}

void CIO::interruptRX()
{

    uint16_t sample = DC_OFFSET;
    uint8_t control = MARK_NONE;
    zmq::message_t mq_message;
    zmq::recv_result_t recv_result = m_zmqsocketRX.recv(mq_message, zmq::recv_flags::none);
    //usleep(500); // RX buffer overflows without the block_size change in IO::process()
    int size = mq_message.size();
    uint32_t data_size = 0;
    if(size < 1)
        return;
    memcpy(&data_size, (unsigned char*)mq_message.data(), sizeof(uint32_t));
    
    ::pthread_mutex_lock(&m_RXlock);
    u_int16_t rx_buf_space = m_rxBuffer.getSpace();
    
    for(int i=0;i < data_size;i++)
    {
        short signed_sample = 0;
        memcpy(&control, (unsigned char*)mq_message.data() + sizeof(uint32_t) + i, sizeof(uint8_t));
        memcpy(&signed_sample, (unsigned char*)mq_message.data() + sizeof(uint32_t) + data_size * sizeof(uint8_t) + i * sizeof(short), sizeof(short));
        m_rxBuffer.put((uint16_t)signed_sample, control);
        m_rssiBuffer.put(3U);
    }
    ::pthread_mutex_unlock(&m_RXlock);
    return;
}

bool CIO::getCOSInt()
{
	return m_COSint;
}

void CIO::setLEDInt(bool on)
{
}

void CIO::setPTTInt(bool on)
{
	//handle enable clock gpio4

}

void CIO::setCOSInt(bool on)
{
    m_COSint = on;
}

void CIO::setDStarInt(bool on)
{
}

void CIO::setDMRInt(bool on)
{
}

void CIO::setYSFInt(bool on)
{
}

void CIO::setP25Int(bool on)
{
}

void CIO::setNXDNInt(bool on)
{
}

void CIO::delayInt(unsigned int dly)
{
  usleep(dly*1000);
}



#endif

