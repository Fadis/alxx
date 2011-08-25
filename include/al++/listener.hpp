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

#ifndef OPENALXX_LISTENER_HPP
#define OPENALXX_LISTENER_HPP

#include <AL/al.h>
#include <AL/alc.h>
#include <algorithm>
#include <boost/type_traits.hpp>
#include <boost/utility/enable_if.hpp>

#include <al++/repack.hpp>
#include <al++/error.hpp>
#include <al++/context.hpp>

namespace al {
  namespace playback {
    extern boost::mutex global_lock;
    
    class Listener : public ContextBoundResource {
    public:
      Listener( Context &_context_core )
      : ContextBoundResource( _context_core ) {
      }
      inline void setGain( ALfloat _gain ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListenerf( AL_GAIN, _gain );
        error::E2E::checkAL();
      }
      inline void setPosition( ALfloat _x, ALfloat _y, ALfloat _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListener3f( AL_POSITION, _x, _y, _z );
        error::E2E::checkAL();
      }
      inline void setPosition( ALint _x, ALint _y, ALint _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListener3i( AL_POSITION, _x, _y, _z );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void setPosition( Iterator _begin, Iterator _end,
                              typename boost::disable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        ALfloat temp[ 3 ];
        repack( temp, temp + 3, _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListenerfv( AL_POSITION, temp );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void setPosition( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        ALint temp[ 3 ];
        repack( temp, temp + 3, _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListeneriv( AL_POSITION, temp );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void setPosition( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALfloat > >::type* = 0,
                              typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        if( std::distance( _begin, _end ) < 3 ) {
          ALfloat temp[ 3 ];
          repack( temp, temp + 3, _begin, _end );
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alListenerfv( AL_POSITION, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alListenerfv( AL_POSITION, _begin );
          error::E2E::checkAL();
        }
      }
      template< typename Iterator >
      inline void setPosition( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        if( std::distance( _begin, _end ) < 3 ) {
          ALint temp[ 3 ];
          repack( temp, temp + 3, _begin, _end );
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alListeneriv( AL_POSITION, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alListeneriv( AL_POSITION, _begin );
          error::E2E::checkAL();
        }
      }
      inline void setVelocity( ALfloat _x, ALfloat _y, ALfloat _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListener3f( AL_VELOCITY, _x, _y, _z );
        error::E2E::checkAL();
      }
      inline void setVelocity( ALint _x, ALint _y, ALint _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListener3i( AL_VELOCITY, _x, _y, _z );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void setVelocity( Iterator _begin, Iterator _end,
                              typename boost::disable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        ALfloat temp[ 3 ];
        repack( temp, temp + 3, _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListenerfv( AL_VELOCITY, temp );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void setVelocity( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        ALint temp[ 3 ];
        repack( temp, temp + 3, _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListeneriv( AL_VELOCITY, temp );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void setVelocity( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALfloat > >::type* = 0,
                              typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        if( std::distance( _begin, _end ) < 3 ) {
          ALfloat temp[ 3 ];
          repack( temp, temp + 3, _begin, _end );
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alListenerfv( AL_VELOCITY, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alListenerfv( AL_POSITION, _begin );
          error::E2E::checkAL();
        }
      }
      template< typename Iterator >
      inline void setVelocity( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        if( std::distance( _begin, _end ) < 3 ) {
          ALint temp[ 3 ];
          repack( temp, temp + 3, _begin, _end );
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alListeneriv( AL_VELOCITY, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alListeneriv( AL_POSITION, _begin );
          error::E2E::checkAL();
        }
      }
      inline void setOrientation( ALfloat _atx, ALfloat _aty, ALfloat _atz, ALfloat _upx, ALfloat _upy, ALfloat _upz ) {
        ALfloat temp[ 6 ] = { _atx, _aty, _atz, _upx, _upy, _upz };
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListenerfv( AL_ORIENTATION, temp );
        error::E2E::checkAL();
      }
      inline void setOrientation( ALint _atx, ALint _aty, ALint _atz, ALint _upx, ALint _upy, ALint _upz ) {
        ALint temp[ 6 ] = { _atx, _aty, _atz, _upx, _upy, _upz };
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListeneriv( AL_ORIENTATION, temp );
        error::E2E::checkAL();
      }
      template< typename AtIterator, typename UpIterator >
      inline void setOrientation( AtIterator _atbegin, AtIterator _atend, UpIterator _upbegin, UpIterator _upend,
                                 typename boost::disable_if< boost::is_same< typename std::iterator_traits< AtIterator >::value_type, ALint > >::type* = 0,
                                 typename boost::disable_if< boost::is_same< typename std::iterator_traits< UpIterator >::value_type, ALint > >::type* = 0 ) {
        ALfloat temp[ 6 ];
        repack( temp, temp + 3, _atbegin, _atend );
        repack( temp + 3, temp + 6, _upbegin, _upend );
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListenerfv( AL_ORIENTATION, temp );
        error::E2E::checkAL();
      }
      template< typename AtIterator, typename UpIterator >
      inline void setOrientation( AtIterator _atbegin, AtIterator _atend, UpIterator _upbegin, UpIterator _upend,
                                 typename boost::enable_if< boost::is_same< typename std::iterator_traits< AtIterator >::value_type, ALint > >::type* = 0,
                                 typename boost::enable_if< boost::is_same< typename std::iterator_traits< UpIterator >::value_type, ALint > >::type* = 0 ) {
        ALint temp[ 6 ];
        repack( temp, temp + 3, _atbegin, _atend );
        repack( temp + 3, temp + 6, _upbegin, _upend );
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alListeneriv( AL_ORIENTATION, temp );
        error::E2E::checkAL();
      }
      inline ALfloat getGain() const {
        ALfloat value;
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetListenerf( AL_GAIN, &value );
        error::E2E::checkAL();
        return value;
      }
      template< typename Iterator >
      inline void getPosition( Iterator _begin, Iterator _end,
                              typename boost::disable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        ALfloat temp[ 3 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListenerfv( AL_POSITION, temp );
          error::E2E::checkAL();
        }
        repack( _begin, _end, temp, temp + 3 );
      }
      template< typename Iterator >
      inline void getPosition( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        ALint temp[ 3 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListeneriv( AL_POSITION, temp );
          error::E2E::checkAL();
        }
        repack( _begin, _end, temp, temp + 3 );
      }
      template< typename Iterator >
      inline void getPosition( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALfloat > >::type* = 0,
                              typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        if( std::distance( _begin, _end ) < 3 ) {
          ALfloat temp[ 3 ];
          {
            boost::mutex::scoped_lock( global_lock );
            getContextCore().makeCurrent();
            alGetListenerfv( AL_POSITION, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListenerfv( AL_POSITION, _begin );
          error::E2E::checkAL();
        }
      }
      template< typename Iterator >
      inline void getPosition( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        if( std::distance( _begin, _end ) < 3 ) {
          ALint temp[ 3 ];
          {
            boost::mutex::scoped_lock( global_lock );
            getContextCore().makeCurrent();
            alGetListeneriv( AL_POSITION, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListeneriv( AL_POSITION, _begin );
          error::E2E::checkAL();
        }
      }
      inline void getPosition( ALfloat &_x, ALfloat &_y, ALfloat &_z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetListener3f( AL_POSITION, &_x, &_y, &_z );
        error::E2E::checkAL();
      }
      inline void getPosition( ALint &_x, ALint &_y, ALint &_z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetListener3i( AL_POSITION, &_x, &_y, &_z );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void getVelocity( Iterator _begin, Iterator _end,
                              typename boost::disable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        ALfloat temp[ 3 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListenerfv( AL_VELOCITY, temp );
          error::E2E::checkAL();
        }
        repack( _begin, _end, temp, temp + 3 );
      }
      template< typename Iterator >
      inline void getVelocity( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        ALint temp[ 3 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListeneriv( AL_VELOCITY, temp );
          error::E2E::checkAL();
        }
        repack( _begin, _end, temp, temp + 3 );
      }
      template< typename Iterator >
      inline void getVelocity( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALfloat > >::type* = 0,
                              typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        if( std::distance( _begin, _end ) < 3 ) {
          ALfloat temp[ 3 ];
          {
            boost::mutex::scoped_lock( global_lock );
            getContextCore().makeCurrent();
            alGetListenerfv( AL_VELOCITY, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListenerfv( AL_VELOCITY, _begin );
          error::E2E::checkAL();
        }
      }
      template< typename Iterator >
      inline void getVelocity( Iterator _begin, Iterator _end,
                              typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        if( std::distance( _begin, _end ) < 3 ) {
          ALint temp[ 3 ];
          {
            boost::mutex::scoped_lock( global_lock );
            getContextCore().makeCurrent();
            alGetListeneriv( AL_VELOCITY, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListeneriv( AL_VELOCITY, _begin );
          error::E2E::checkAL();
        }
      }
      inline void getVelocity( ALfloat &_x, ALfloat &_y, ALfloat &_z ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetListener3f( AL_VELOCITY, &_x, &_y, &_z );
        error::E2E::checkAL();
      }
      inline void getVelocity( ALint &_x, ALint &_y, ALint &_z ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetListener3i( AL_VELOCITY, &_x, &_y, &_z );
        error::E2E::checkAL();
      }
      template< typename AtIterator, typename UpIterator >
      inline void getOrientation( AtIterator _atbegin, AtIterator _atend, UpIterator _upbegin, UpIterator _upend,
                                 typename boost::disable_if< boost::is_same< typename std::iterator_traits< AtIterator >::value_type, ALint > >::type* = 0,
                                 typename boost::disable_if< boost::is_same< typename std::iterator_traits< UpIterator >::value_type, ALint > >::type* = 0 ) const {
        ALfloat temp[ 6 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListenerfv( AL_ORIENTATION, temp );
          error::E2E::checkAL();
        }
        repack( _atbegin, _atend, temp, temp + 3 );
        repack( _upbegin, _upend, temp + 3, temp + 6 );
      }
      template< typename AtIterator, typename UpIterator >
      inline void getOrientation( AtIterator _atbegin, AtIterator _atend, UpIterator _upbegin, UpIterator _upend,
                                 typename boost::enable_if< boost::is_same< typename std::iterator_traits< AtIterator >::value_type, ALint > >::type* = 0,
                                 typename boost::enable_if< boost::is_same< typename std::iterator_traits< UpIterator >::value_type, ALint > >::type* = 0 ) const {
        ALint temp[ 6 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListeneriv( AL_ORIENTATION, temp );
          error::E2E::checkAL();
        }
        repack( _atbegin, _atend, temp, temp + 3 );
        repack( _upbegin, _upend, temp + 3, temp + 6 );
      }
      inline void getOrientation( ALfloat &_atx, ALfloat &_aty, ALfloat &_atz, ALfloat &_upx, ALfloat &_upy, ALfloat &_upz ) const {
        ALfloat temp[ 6 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListenerfv( AL_ORIENTATION, temp );
          error::E2E::checkAL();
        }
        _atx = temp[ 0 ];
        _aty = temp[ 1 ];
        _atz = temp[ 2 ];
        _upx = temp[ 3 ];
        _upy = temp[ 4 ];
        _upz = temp[ 5 ];
      }
      inline void getOrientation( ALint &_atx, ALint &_aty, ALint &_atz, ALint &_upx, ALint &_upy, ALint &_upz ) const {
        ALint temp[ 6 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetListeneriv( AL_ORIENTATION, temp );
          error::E2E::checkAL();
        }
        _atx = temp[ 0 ];
        _aty = temp[ 1 ];
        _atz = temp[ 2 ];
        _upx = temp[ 3 ];
        _upy = temp[ 4 ];
        _upz = temp[ 5 ];
      }
    };
  }
}

#endif
