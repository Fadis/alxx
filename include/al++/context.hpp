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

#ifndef OPENALXX_CONTEXT_HPP
#define OPENALXX_CONTEXT_HPP

#include <AL/al.h>
#include <AL/alc.h>
#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <al++/repack.hpp>
#include <al++/error.hpp>
#include <al++/device.hpp>

namespace al {
  namespace playback {
    extern boost::mutex global_lock;
    
    class ContextAttribute {
    public:
      ContextAttribute(){
      }
      inline void setFrequency( ALCint _value ) {
        frequency = _value;
      }
      inline void unsetFrequency() {
        frequency = boost::optional< ALCint >();
      }
      inline boost::optional< ALCint > getFrequency() const {
        return frequency;
      }
      inline void setRefresh( ALCint _value ) {
        refresh = _value;
      }
      inline void unsetRefresh() {
        refresh = boost::optional< ALCint >();
      }
      inline boost::optional< ALCint > getRefresh() const {
        return refresh;
      }
      inline void setSync( ALCint _value ) {
        sync = _value;
      }
      inline void unsetSync() {
        sync = boost::optional< ALCint >();
      }
      inline boost::optional< ALCint > getSync() const {
        return sync;
      }
      template< typename Iterator >
      void getAttrList( Iterator _begin, Iterator _end ) const {
        ALCint attr_list[ 7 ];
        ALCint *cur = attr_list;
        if( frequency ) {
          cur[ 0 ] = ALC_FREQUENCY;
          cur[ 1 ] = *frequency;
          cur += 2;
        }
        if( refresh ) {
          cur[ 0 ] = ALC_REFRESH;
          cur[ 1 ] = *refresh;
          cur += 2;
        }
        if( sync ) {
          cur[ 0 ] = ALC_REFRESH;
          cur[ 1 ] = *sync;
          cur += 2;
        }
        cur[ 0 ] = 0;
        cur++;
        repack( _begin, _end, attr_list, cur );
      }
    private:
      boost::optional< ALCint > frequency;
      boost::optional< ALCint > refresh;
      boost::optional< ALCint > sync;
    };
    
    
    
    class ContextCore {
    public:
      ContextCore( const Device &_device, const boost::shared_ptr< ALCcontext > &_raw_context )
      : device( _device ), raw_context( _raw_context ) {
      }
      inline Device getDevice() const {
        return device;
      }
      inline void makeCurrent() const {
        if( !isCurrent() ) {
          alcMakeContextCurrent( raw_context.get() );
          error::E2E::checkALC( device.getRawDevice().get() );
        }
      }
      inline bool isCurrent() const {
        return alcGetCurrentContext() == raw_context.get();
      }
      inline bool isExtensionPresent( const ALCchar *_name ) const {
        boost::mutex::scoped_lock( global_lock );
        makeCurrent();
        bool result = alIsExtensionPresent( _name ) == ALC_TRUE;
        error::E2E::checkAL();
        return result;
      }
      inline void *getProcAddress( const ALCchar *_name ) const {
        boost::mutex::scoped_lock( global_lock );
        makeCurrent();
        void *func_addr = alGetProcAddress( _name );
        error::E2E::checkAL();
        return func_addr;
      }
      inline ALCenum getEnumValue( const ALCchar *_name ) const {
        boost::mutex::scoped_lock( global_lock );
        makeCurrent();
        ALCenum value = alGetEnumValue( _name );
        error::E2E::checkAL();
        return value;
      }
    private:
      Device device;
      boost::shared_ptr< ALCcontext > raw_context;
    };
    
    class Context {
    public:
      Context( Device &_device );      
      Context( Device &_device, const ContextAttribute &_attr );      
      Context( const Context &_source );
      inline void wakeup() {
        boost::mutex::scoped_lock( global_lock );
        alcProcessContext( raw_context.get() );
        error::E2E::checkALC( device.getRawDevice().get() );
      }
      inline void suspend() {
        boost::mutex::scoped_lock( global_lock );
        alcSuspendContext( raw_context.get() );
        error::E2E::checkALC( device.getRawDevice().get() );
      }
      inline Device getDevice() const {
        return device;
      }
      inline ContextCore getCore() const {
        ContextCore temp( device, raw_context );
        return temp;
      }
    private:
      static void deleter( ALCcontext *_context, Device _device );
      boost::optional< ContextAttribute > attr;
      Device device;
      boost::shared_ptr< ALCcontext > raw_context;
    };
    
    class ALCExtensionList : public std::vector< std::string > {
    public:
      ALCExtensionList( Device &_device );
    };

    std::ostream & operator<<( std::ostream  &_stream , ALCExtensionList &_ext ) {
      BOOST_FOREACH( const std::string &elem, _ext )
        _stream << elem  << std::endl;
      return _stream;;
    }
    
    class ALExtensionList : public std::vector< std::string > {
    public:
      ALExtensionList( Context &_context );
    };

    std::ostream & operator<<( std::ostream  &_stream , ALExtensionList &_ext ) {
      BOOST_FOREACH( const std::string &elem, _ext )
        _stream << elem  << std::endl;
      return _stream;;
    }

    class ContextBoundResource {
    public:
      ContextBoundResource( const Context &_context )
      : context_core( _context.getCore() ) {
      }
      ContextBoundResource( const ContextCore &_context_core )
      : context_core( _context_core ) {
      }
      inline ContextCore &getContextCore() {
        return context_core;
      }
      inline const ContextCore &getContextCore() const {
        return context_core;
      }
    private:
      ContextCore context_core;
    };
    
  }
}

#endif
