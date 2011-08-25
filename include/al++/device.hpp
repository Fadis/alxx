/***************************************************************************
 *   Copyright (C) 2011 by Naomasa Matsubayashi   *
 *   fadis@quaternion.sakura.ne.jp   *
 *                                                                         *
 *   All rights reserved.                                                  *
 *                                                                         *
 * Redistribution and use in source and binary forms, with or without      *
 * modification, are permitted provided that the following conditions are  *
 * met:                                                                    *
 *                                                                         *
 *  1. Redistributions of source code must retain the above copyright      *
 *     notice, this list of conditions and the following disclaimer.       *
 *  2. Redistributions in binary form must reproduce the above copyright   *
 *     notice, this list of conditions and the following disclaimer in the *
 *     documentation and/or other materials provided with the distribution.*
 *                                                                         *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT      *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ***************************************************************************/

#ifndef OPENALXX_DEVICE_HPP
#define OPENALXX_DEVICE_HPP

#include <AL/al.h>
#include <AL/alc.h>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <al++/singleton.hpp>
#include <al++/repack.hpp>
#include <al++/error.hpp>

namespace al {
  namespace playback {
    extern boost::mutex global_lock;
    
    class DeviceList : public std::vector< const char * > {
    public:
      DeviceList();
    };
    
    class DeviceManager {
    private:
      friend class Singleton< DeviceManager >;
      typedef std::map< const char *, std::pair< ALCdevice*, unsigned int > > InternalContainer;
    public:
      ALCdevice *open( const char *_device_name );
      void close( ALCdevice *_device, const char *_device_name );
    private:
      DeviceManager() {}
      InternalContainer opened_devices;
      boost::mutex map_guard;
    };
    
    class Device {
    public:
      Device( const char *_device_name ) :
      raw_device( Singleton< DeviceManager >::get().open( _device_name ),
                 boost::bind( &DeviceManager::close, &Singleton< DeviceManager >::get(), _1, _device_name ) ) {
      }
      Device( const boost::shared_ptr< ALCdevice > &_raw_device ) {
        raw_device = _raw_device;
      }
      inline void getVersion( ALCint &_major, ALCint &_minor ) {
        boost::mutex::scoped_lock( global_lock );
        alcGetIntegerv( raw_device.get(), ALC_MAJOR_VERSION, 1, &_major );
        error::E2E::checkALC( raw_device.get() );
        alcGetIntegerv( raw_device.get(), ALC_MINOR_VERSION, 1, &_minor );
        error::E2E::checkALC( raw_device.get() );
      }
      template< typename Iterator >
      inline void getVersion( Iterator _begin, Iterator _end ) {
        ALCint temp[ 2 ];
        {
          boost::mutex::scoped_lock( global_lock );
          alcGetIntegerv( raw_device.get(), ALC_MAJOR_VERSION, 1, temp );
          error::E2E::checkALC( raw_device.get() );
          alcGetIntegerv( raw_device.get(), ALC_MINOR_VERSION, 1, temp + 1 );
          error::E2E::checkALC( raw_device.get() );
        }
        repack( _begin, _end, temp, temp + 2 );
      }
      inline bool isExtensionPresent( const ALCchar *_name ) {
        boost::mutex::scoped_lock( global_lock );
        bool result = alcIsExtensionPresent( raw_device.get(), _name ) == ALC_TRUE;
        error::E2E::checkALC( raw_device.get() );
        return result;
      }
      inline void *getProcAddress( const ALCchar *_name ) {
        boost::mutex::scoped_lock( global_lock );
        void *func_addr = alcGetProcAddress( raw_device.get(), _name );
        error::E2E::checkALC( raw_device.get() );
        return func_addr;
      }
      inline ALCenum getEnumValue( const ALCchar *_name ) {
        boost::mutex::scoped_lock( global_lock );
        ALCenum value = alcGetEnumValue( raw_device.get(), _name );
        error::E2E::checkALC( raw_device.get() );
        return value;
      }
      boost::shared_ptr< ALCdevice > getRawDevice() const { return raw_device; }
    private:
      boost::shared_ptr< ALCdevice > raw_device;
    };
  }
}

#endif
