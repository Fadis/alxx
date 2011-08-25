/***************************************************************************
 *   Copyright (C) 2010 by Naomasa Matsubayashi   *
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

#ifndef OPENALXX_BUFFER_HPP
#define OPENALXX_BUFFER_HPP

#include <AL/al.h>
#include <AL/alc.h>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/type_traits.hpp>
#include <boost/utility/enable_if.hpp>

#include <al++/repack.hpp>
#include <al++/error.hpp>
#include <al++/context.hpp>

namespace al {
  namespace playback {
    extern boost::mutex global_lock;
    
    class BufferCore : public ContextBoundResource {
    public:
      BufferCore( Context &_context, ALuint _buffer )
      : ContextBoundResource( _context ), buffer_deleter( static_cast<void*>( 0 ), boost::bind( &BufferCore::deleter, _1, getContextCore(), _buffer ) ), buffer( _buffer ) {
      }
      BufferCore( ContextCore &_context_core, ALuint _buffer )
      : ContextBoundResource( _context_core ), buffer_deleter( static_cast<void*>( 0 ), boost::bind( &BufferCore::deleter, _1, getContextCore(), _buffer ) ), buffer( _buffer ) {
      }
      inline ALuint get() const {
        return buffer;
      }
      inline bool unique() const {
        return buffer_deleter.unique();
      }
    private:
      static void deleter( void *_dummy, ContextCore &_context_core, ALuint _buffer );
      boost::shared_ptr< void > buffer_deleter;
      ALuint buffer;
    };
    
    class Buffer {
    public:
      Buffer( Context &_context ) : buffer_core( Buffer::initCore( _context ) ) {
      }
      Buffer( ContextCore &_context_core ) : buffer_core( Buffer::initCore( _context_core ) ) {
      }
      Buffer( const BufferCore &_buffer_core )
      : buffer_core( _buffer_core ) {
      }
      template< typename Iterator >
      void setData( Iterator _begin, Iterator _end, ALsizei _freq,
                   typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALdouble > >::type* = 0,
                   typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        unlinkShared();
        bool extension_is_available = buffer_core.getContextCore().isExtensionPresent( "AL_EXT_DOUBLE" );
        if( !extension_is_available )
          throw error::ExtensionNotAvailable();
        unsigned int data_size = std::distance( _begin, _end );
        std::vector< ALfloat > temp;
        temp.resize( data_size );
        repack( temp.begin(), temp.end(), _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        buffer_core.getContextCore().makeCurrent();
        alBufferData( buffer_core.get(), buffer_core.getContextCore().getEnumValue( "AL_FORMAT_MONO_DOUBLE" ), static_cast< void* >( &temp.front() ), data_size * sizeof( ALdouble ), _freq );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      void setData( Iterator _begin, Iterator _end, ALsizei _freq,
                   typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALfloat > >::type* = 0,
                   typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        unlinkShared();
        bool extension_is_available = buffer_core.getContextCore().isExtensionPresent( "AL_EXT_FLOAT32" );
        if( !extension_is_available )
          throw error::ExtensionNotAvailable();
        unsigned int data_size = std::distance( _begin, _end );
        std::vector< ALfloat > temp;
        temp.resize( data_size );
        repack( temp.begin(), temp.end(), _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        buffer_core.getContextCore().makeCurrent();
        alBufferData( buffer_core.get(), buffer_core.getContextCore().getEnumValue( "AL_FORMAT_MONO_FLOAT32" ), static_cast< void* >( &temp.front() ), data_size * sizeof( ALfloat ), _freq );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      void setData( Iterator _begin, Iterator _end, ALsizei _freq,
                   typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALshort > >::type* = 0,
                   typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        unlinkShared();
        unsigned int data_size = std::distance( _begin, _end );
        std::vector< ALshort > temp;
        temp.resize( data_size );
        repack( temp.begin(), temp.end(), _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        buffer_core.getContextCore().makeCurrent();
        alBufferData( buffer_core.get(), AL_FORMAT_MONO16, static_cast< void* >( &temp.front() ), data_size * sizeof( ALshort ), _freq );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      void setData( Iterator _begin, Iterator _end, ALsizei _freq,
                   typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALbyte > >::type* = 0,
                   typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        unlinkShared();
        unsigned int data_size = std::distance( _begin, _end );
        std::vector< ALbyte > temp;
        temp.resize( data_size );
        repack( temp.begin(), temp.end(), _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        buffer_core.getContextCore().makeCurrent();
        alBufferData( buffer_core.get(), AL_FORMAT_MONO8, static_cast< void* >( &temp.front() ), data_size * sizeof( ALbyte ), _freq );
        error::E2E::checkAL();
      }
      
      template< typename Iterator >
      void setData( Iterator _begin, Iterator _end, ALsizei _freq,
                   typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALdouble > >::type* = 0,
                   typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        unlinkShared();
        bool extension_is_available = buffer_core.getContextCore().isExtensionPresent( "AL_EXT_DOUBLE" );
        if( !extension_is_available )
          throw error::ExtensionNotAvailable();
        unsigned int data_size = std::distance( _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        buffer_core.getContextCore().makeCurrent();
        alBufferData( buffer_core.get(),  buffer_core.getContextCore().getEnumValue( "AL_FORMAT_MONO_DOUBLE" ), static_cast< const void* >( _begin ), data_size * sizeof( ALdouble ), _freq );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      void setData( Iterator _begin, Iterator _end, ALsizei _freq,
                   typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALfloat > >::type* = 0,
                   typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        unlinkShared();
        bool extension_is_available = buffer_core.getContextCore().isExtensionPresent( "AL_EXT_FLOAT32" );
        if( !extension_is_available )
          throw error::ExtensionNotAvailable();
        unsigned int data_size = std::distance( _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        buffer_core.getContextCore().makeCurrent();
        alBufferData( buffer_core.get(), buffer_core.getContextCore().getEnumValue( "AL_FORMAT_MONO_FLOAT32" ), static_cast< const void* >( _begin ), data_size * sizeof( ALfloat ), _freq );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      void setData( Iterator _begin, Iterator _end, ALsizei _freq,
                   typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALshort > >::type* = 0,
                   typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        unlinkShared();
        unsigned int data_size = std::distance( _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        buffer_core.getContextCore().makeCurrent();
        alBufferData( buffer_core.get(), AL_FORMAT_MONO16, static_cast< const void* >( _begin ), data_size * sizeof( ALshort ), _freq );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      void setData( Iterator _begin, Iterator _end, ALsizei _freq,
                   typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALbyte > >::type* = 0,
                   typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        unlinkShared();
        unsigned int data_size = std::distance( _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        buffer_core.getContextCore().makeCurrent();
        alBufferData( buffer_core.get(), AL_FORMAT_MONO8, static_cast< const void* >( _begin ), data_size * sizeof( ALbyte ), _freq );
        error::E2E::checkAL();
      }
      inline BufferCore &getCore() {
        return buffer_core;
      }
      inline const BufferCore &getCore() const {
        return buffer_core;
      }
      inline void setCore( BufferCore &_buffer_core ) {
        buffer_core = _buffer_core;
      }
    private:
      inline void unlinkShared() {
        if( !buffer_core.unique() ) {
          ALuint buffer;
          {
            boost::mutex::scoped_lock( global_lock );
            buffer_core.getContextCore().makeCurrent();
            alGenBuffers( 1, &buffer );
            error::E2E::checkAL();
          }
          buffer_core = BufferCore( buffer_core.getContextCore(), buffer );
        }
      }
      inline static BufferCore initCore( Context &_context ) {
        ALuint buffer;
        {
          boost::mutex::scoped_lock( global_lock );
          _context.getCore().makeCurrent();
          alGenBuffers( 1, &buffer );
          error::E2E::checkAL();
        }
        return BufferCore( _context, buffer );
      }
      inline static BufferCore initCore( ContextCore &_context_core ) {
        ALuint buffer;
        {
          boost::mutex::scoped_lock( global_lock );
          _context_core.makeCurrent();
          alGenBuffers( 1, &buffer );
          error::E2E::checkAL();
        }
        return BufferCore( _context_core, buffer );
      }
      BufferCore buffer_core;
    };
  }
}

#endif

