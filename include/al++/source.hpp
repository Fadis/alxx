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

#ifndef OPENALXX_SOURCE_HPP
#define OPENALXX_SOURCE_HPP

#include <AL/al.h>
#include <AL/alc.h>
#include <list>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/type_traits.hpp>
#include <boost/utility/enable_if.hpp>

#include <al++/repack.hpp>
#include <al++/error.hpp>
#include <al++/context.hpp>
#include <al++/buffer.hpp>

namespace al {
  namespace playback {
    extern boost::mutex global_lock;
    class Source : public ContextBoundResource, private std::list< BufferCore > {
    public:
      Source( Context &_context );
      Source( const Source &_source );
      inline void looping( bool _loop ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        if( _loop )
          alSourcei( source, AL_LOOPING, AL_TRUE );
        else
          alSourcei( source, AL_LOOPING, AL_FALSE );
        error::E2E::checkAL();
      }
      inline void listenerRelative( bool _relative ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        if( _relative )
          alSourcei( source, AL_SOURCE_RELATIVE, AL_TRUE );
        else
          alSourcei( source, AL_SOURCE_RELATIVE, AL_FALSE );
        error::E2E::checkAL();
      }
      inline void setGain( ALfloat _gain ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcef( source, AL_GAIN, _gain );
        error::E2E::checkAL();
      }
      inline void setGain( ALfloat _min, ALfloat _max ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcef( source, AL_MIN_GAIN, _min );
        error::E2E::checkAL();
        alSourcef( source, AL_MAX_GAIN, _max );
        error::E2E::checkAL();
      }
      inline void setPitch( ALfloat _pitch ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcef( source, AL_PITCH, _pitch );
        error::E2E::checkAL();
      }
      inline void setPosition( ALfloat _x, ALfloat _y, ALfloat _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSource3f( source, AL_POSITION, _x, _y, _z );
        error::E2E::checkAL();
      }
      inline void setPosition( ALint _x, ALint _y, ALint _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSource3i( source, AL_POSITION, _x, _y, _z );
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
        alSourcefv( source, AL_POSITION, temp );
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
        alSourceiv( source, AL_POSITION, temp );
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
          alSourcefv( source, AL_POSITION, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourcefv( source, AL_POSITION, _begin );
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
          alSourceiv( source, AL_POSITION, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourceiv( source, AL_POSITION, _begin );
          error::E2E::checkAL();
        }
      }
      inline void setVelocity( ALfloat _x, ALfloat _y, ALfloat _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSource3f( source, AL_VELOCITY, _x, _y, _z );
        error::E2E::checkAL();
      }
      inline void setVelocity( ALint _x, ALint _y, ALint _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSource3i( source, AL_VELOCITY, _x, _y, _z );
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
        alSourcefv( source, AL_VELOCITY, temp );
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
        alSourceiv( source, AL_VELOCITY, temp );
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
          alSourcefv( source, AL_VELOCITY, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourcefv( source, AL_VELOCITY, _begin );
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
          alSourceiv( source, AL_VELOCITY, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourceiv( source, AL_VELOCITY, _begin );
          error::E2E::checkAL();
        }
      }
      inline void setDirection( ALfloat _x, ALfloat _y, ALfloat _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSource3f( source, AL_DIRECTION, _x, _y, _z );
        error::E2E::checkAL();
      }
      inline void setDirection( ALint _x, ALint _y, ALint _z ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSource3i( source, AL_DIRECTION, _x, _y, _z );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void setDirection( Iterator _begin, Iterator _end,
                               typename boost::disable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                               typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        ALfloat temp[ 3 ];
        repack( temp, temp + 3, _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcefv( source, AL_DIRECTION, temp );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void setDirection( Iterator _begin, Iterator _end,
                               typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                               typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        ALint temp[ 3 ];
        repack( temp, temp + 3, _begin, _end );
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourceiv( source, AL_DIRECTION, temp );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void setDirection( Iterator _begin, Iterator _end,
                               typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALfloat > >::type* = 0,
                               typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        if( std::distance( _begin, _end ) < 3 ) {
          ALfloat temp[ 3 ];
          repack( temp, temp + 3, _begin, _end );
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourcefv( source, AL_DIRECTION, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourcefv( source, AL_DIRECTION, _begin );
          error::E2E::checkAL();
        }
      }
      template< typename Iterator >
      inline void setDirection( Iterator _begin, Iterator _end,
                               typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                               typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
        if( std::distance( _begin, _end ) < 3 ) {
          ALint temp[ 3 ];
          repack( temp, temp + 3, _begin, _end );
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourceiv( source, AL_DIRECTION, temp );
          error::E2E::checkAL();
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourceiv( source, AL_DIRECTION, _begin );
          error::E2E::checkAL();
        }
      }
      inline void setDistanceModel( ALenum _type, ALfloat _reference_distance,
                                   ALfloat _max_distance, ALfloat _rolloff_factor ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcef( source, AL_REFERENCE_DISTANCE, _reference_distance );
        error::E2E::checkAL();
        alSourcef( source, AL_MAX_DISTANCE, _max_distance );
        error::E2E::checkAL();
        alSourcef( source, AL_ROLLOFF_FACTOR, _rolloff_factor );
        error::E2E::checkAL();
        alDistanceModel( _type );
        distance_model = _type;
        error::E2E::checkAL();
      }
      inline void setCone( ALfloat _inner_angle, ALfloat _outer_angle, ALfloat _outer_gain ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcef( source, AL_CONE_INNER_ANGLE, _inner_angle );
        error::E2E::checkAL();
        alSourcef( source, AL_CONE_OUTER_ANGLE, _outer_angle );
        error::E2E::checkAL();
        alSourcef( source, AL_CONE_OUTER_GAIN, _outer_gain );
        error::E2E::checkAL();
      }
      inline void setCone( ALint _inner_angle, ALint _outer_angle, ALfloat _outer_gain ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcei( source, AL_CONE_INNER_ANGLE, _inner_angle );
        error::E2E::checkAL();
        alSourcei( source, AL_CONE_OUTER_ANGLE, _outer_angle );
        error::E2E::checkAL();
        alSourcef( source, AL_CONE_OUTER_GAIN, _outer_gain );
        error::E2E::checkAL();
      }
      inline void setSecOffset( ALfloat _offset ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcef( source, AL_SEC_OFFSET, _offset );
        error::E2E::checkAL();
      }
      inline void setSecOffset( ALint _offset ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcei( source, AL_SEC_OFFSET, _offset );
        error::E2E::checkAL();
      }
      inline void setSampleOffset( ALfloat _offset ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcef( source, AL_SAMPLE_OFFSET, _offset );
        error::E2E::checkAL();
      }
      inline void setSampleOffset( ALint _offset ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcei( source, AL_SAMPLE_OFFSET, _offset );
        error::E2E::checkAL();
      }
      inline void setByteOffset( ALfloat _offset ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcef( source, AL_BYTE_OFFSET, _offset );
        error::E2E::checkAL();
      }
      inline void setByteOffset( ALint _offset ) {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcei( source, AL_BYTE_OFFSET, _offset );
        error::E2E::checkAL();
      }
      inline bool isLooping() const {
        ALint loop;
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_LOOPING, &loop );
        error::E2E::checkAL();
        return loop == AL_TRUE;
      }
      inline bool isListenerRelative() const {
        ALint relative;
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_LOOPING, &relative );
        error::E2E::checkAL();
        return relative == AL_TRUE;
      }
      inline void getGain( ALfloat &_gain ) const {
        boost::mutex::scoped_lock( global_lock );
        alGetSourcef( source, AL_GAIN, &_gain );
        error::E2E::checkAL();
      }
      inline void getGain( ALfloat &_min, ALfloat &_max ) const {
        boost::mutex::scoped_lock( global_lock );
        alGetSourcef( source, AL_MIN_GAIN, &_min );
        error::E2E::checkAL();
        alGetSourcef( source, AL_MAX_GAIN, &_max );
        error::E2E::checkAL();
      }
      inline void getPitch( ALfloat &_pitch ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcef( source, AL_PITCH, &_pitch );
        error::E2E::checkAL();
      }
      inline void getPosition( ALfloat &_x, ALfloat &_y, ALfloat &_z ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSource3f( source, AL_POSITION, &_x, &_y, &_z );
        error::E2E::checkAL();
      }
      inline void getPosition( ALint &_x, ALint &_y, ALint &_z ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSource3i( source, AL_POSITION, &_x, &_y, &_z );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void getPosition( Iterator _begin, Iterator _end,
                              typename boost::disable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                              typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        ALfloat temp[ 3 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetSourcefv( source, AL_POSITION, temp );
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
          alGetSourceiv( source, AL_POSITION, temp );
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
            alGetSourcefv( source, AL_POSITION, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetSourcefv( source, AL_POSITION, _begin );
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
            alGetSourceiv( source, AL_POSITION, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetSourceiv( source, AL_POSITION, _begin );
          error::E2E::checkAL();
        }
      }
      inline void getVelocity( ALfloat &_x, ALfloat &_y, ALfloat &_z ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSource3f( source, AL_VELOCITY, &_x, &_y, &_z );
        error::E2E::checkAL();
      }
      inline void getVelocity( ALint &_x, ALint &_y, ALint &_z ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSource3i( source, AL_VELOCITY, &_x, &_y, &_z );
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
          alGetSourcefv( source, AL_VELOCITY, temp );
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
          alGetSourceiv( source, AL_VELOCITY, temp );
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
            alGetSourcefv( source, AL_VELOCITY, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetSourcefv( source, AL_VELOCITY, _begin );
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
            alGetSourceiv( source, AL_VELOCITY, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetSourceiv( source, AL_VELOCITY, _begin );
          error::E2E::checkAL();
        }
      }
      inline void getDirection( ALfloat &_x, ALfloat &_y, ALfloat &_z ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSource3f( source, AL_DIRECTION, &_x, &_y, &_z );
        error::E2E::checkAL();
      }
      inline void getDirection( ALint &_x, ALint &_y, ALint &_z ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSource3i( source, AL_DIRECTION, &_x, &_y, &_z );
        error::E2E::checkAL();
      }
      template< typename Iterator >
      inline void getDirection( Iterator _begin, Iterator _end,
                               typename boost::disable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                               typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        ALfloat temp[ 3 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetSourcefv( source, AL_DIRECTION, temp );
          error::E2E::checkAL();
        }
        repack( _begin, _end, temp, temp + 3 );
      }
      template< typename Iterator >
      inline void getDirection( Iterator _begin, Iterator _end,
                               typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                               typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        ALint temp[ 3 ];
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetSourceiv( source, AL_DIRECTION, temp );
          error::E2E::checkAL();
        }
        repack( _begin, _end, temp, temp + 3 );
      }
      template< typename Iterator >
      inline void getDirection( Iterator _begin, Iterator _end,
                               typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALfloat > >::type* = 0,
                               typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        if( std::distance( _begin, _end ) < 3 ) {
          ALfloat temp[ 3 ];
          {
            boost::mutex::scoped_lock( global_lock );
            getContextCore().makeCurrent();
            alGetSourcefv( source, AL_DIRECTION, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetSourcefv( source, AL_DIRECTION, _begin );
          error::E2E::checkAL();
        }
      }
      template< typename Iterator >
      inline void getDirection( Iterator _begin, Iterator _end,
                               typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, ALint > >::type* = 0,
                               typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) const {
        if( std::distance( _begin, _end ) < 3 ) {
          ALint temp[ 3 ];
          {
            boost::mutex::scoped_lock( global_lock );
            getContextCore().makeCurrent();
            alGetSourceiv( source, AL_DIRECTION, temp );
            error::E2E::checkAL();
          }
          repack( _begin, _end, temp, temp + 3 );
        }
        else {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alGetSourceiv( source, AL_DIRECTION, _begin );
          error::E2E::checkAL();
        }
      }
      inline void getDistanceModel( ALenum &_type, ALfloat &_reference_distance,
                                   ALfloat &_max_distance, ALfloat &_rolloff_factor ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcef( source, AL_REFERENCE_DISTANCE, &_reference_distance );
        error::E2E::checkAL();
        alGetSourcef( source, AL_MAX_DISTANCE, &_max_distance );
        error::E2E::checkAL();
        alGetSourcef( source, AL_ROLLOFF_FACTOR, &_rolloff_factor );
        error::E2E::checkAL();
        _type = distance_model;
      }
      inline void getCone( ALfloat &_inner_angle, ALfloat &_outer_angle, ALfloat &_outer_gain ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcef( source, AL_CONE_INNER_ANGLE, &_inner_angle );
        error::E2E::checkAL();
        alGetSourcef( source, AL_CONE_OUTER_ANGLE, &_outer_angle );
        error::E2E::checkAL();
        alGetSourcef( source, AL_CONE_OUTER_GAIN, &_outer_gain );
        error::E2E::checkAL();
      }
      inline void getCone( ALint &_inner_angle, ALint &_outer_angle, ALfloat &_outer_gain ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_CONE_INNER_ANGLE, &_inner_angle );
        error::E2E::checkAL();
        alGetSourcei( source, AL_CONE_OUTER_ANGLE, &_outer_angle );
        error::E2E::checkAL();
        alGetSourcef( source, AL_CONE_OUTER_GAIN, &_outer_gain );
        error::E2E::checkAL();
      }      
      inline void getState( ALint &_state ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_SOURCE_STATE, &_state );
        error::E2E::checkAL();
      }
      inline void getQueuedCount( ALint &_count ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_BUFFERS_QUEUED, &_count );
        error::E2E::checkAL();
      }
      inline void getProcessedCount( ALint &_count ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_BUFFERS_PROCESSED, &_count );
        error::E2E::checkAL();
      }
      inline ALint getState() const {
        ALint temp;
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_SOURCE_STATE, &temp );
        error::E2E::checkAL();
        return temp;
      }
      inline ALint getQueuedCount() const {
        ALint temp;
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_BUFFERS_QUEUED, &temp );
        error::E2E::checkAL();
        return temp;
      }
      inline ALint getProcessedCount() const {
        ALint temp;
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_BUFFERS_PROCESSED, &temp );
        error::E2E::checkAL();
        return temp;
      }
      inline void getSecOffset( ALfloat &_offset ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcef( source, AL_SEC_OFFSET, &_offset );
        error::E2E::checkAL();
      }
      inline void getSecOffset( ALint &_offset ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_SEC_OFFSET, &_offset );
        error::E2E::checkAL();
      }
      inline void getSampleOffset( ALfloat &_offset ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcef( source, AL_SAMPLE_OFFSET, &_offset );
        error::E2E::checkAL();
      }
      inline void getSampleOffset( ALint &_offset ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_SAMPLE_OFFSET, &_offset );
        error::E2E::checkAL();
      }
      inline void getByteOffset( ALfloat &_offset ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcef( source, AL_BYTE_OFFSET, &_offset );
        error::E2E::checkAL();
      }
      inline void getByteOffset( ALint &_offset ) const {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGetSourcei( source, AL_BYTE_OFFSET, &_offset );
        error::E2E::checkAL();
      }
      inline void queueBuffer( Buffer &_buffer ) {
        ALuint raw_buffer = _buffer.getCore().get();
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourceQueueBuffers( source, 1, &raw_buffer );
          error::E2E::checkAL();
        }
        push_front( _buffer.getCore() );
      }
      inline void dequeueBuffer( Buffer &_buffer ) {
        ALuint raw_buffer;
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourceUnqueueBuffers( source, 1, &raw_buffer );
          error::E2E::checkAL();
        }
        _buffer.setCore( back() );
        pop_back();
      }
      inline Buffer dequeueBuffer() {
        ALuint raw_buffer;
        {
          boost::mutex::scoped_lock( global_lock );
          getContextCore().makeCurrent();
          alSourceUnqueueBuffers( source, 1, &raw_buffer );
          error::E2E::checkAL();
        }
        Buffer temp( back() );
        pop_back();
        return temp;
      }
      inline void play() {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcePlay( source );
        error::E2E::checkAL();
      }
      inline void pause() {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourcePause( source );
        error::E2E::checkAL();
      }
      inline void stop() {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alSourceStop( source );
        error::E2E::checkAL();
      }
      inline ALuint getRawSource() const {
        return source;
      }
      void wait() const {
        ALint stat;
        do {
          {
            boost::mutex::scoped_lock( global_lock );
            getContextCore().makeCurrent();
            alGetSourcei( source, AL_SOURCE_STATE, &stat );
          }
          boost::thread::yield();
        } while( stat == AL_PLAYING );
      }
    private:
      static void deleter( void *_dummy, ContextCore &_context_core, ALuint _source );
      boost::shared_ptr< void > source_deleter;
      ALuint source;
      ALenum distance_model;
    };
    
    template< typename Iterator >
    inline void play( Iterator _begin, Iterator _end,
                     typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, Source > >::type* = 0,
                     typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
      std::vector< ALuint > temp;
      Iterator iter;
      for( iter = _begin; iter != _end; iter++ )
        temp.push_back( iter->getRawSource() );
      boost::mutex::scoped_lock( global_lock );
      _begin->getContextCore().makeCurrent();
      alSourcePlayv( temp.size(), &temp.front() );
      error::E2E::checkAL();
    }
    template< typename Iterator >
    inline void pause( Iterator _begin, Iterator _end,
                      typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, Source > >::type* = 0,
                      typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
      std::vector< ALuint > temp;
      Iterator iter;
      for( iter = _begin; iter != _end; iter++ )
        temp.push_back( iter->getRawSource() );
      boost::mutex::scoped_lock( global_lock );
      _begin->getContextCore().makeCurrent();
      alSourcePausev( temp.size(), &temp.front() );
      error::E2E::checkAL();
    }
    template< typename Iterator >
    inline void stop( Iterator _begin, Iterator _end,
                     typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, Source > >::type* = 0,
                     typename boost::disable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
      std::vector< ALuint > temp;
      Iterator iter;
      for( iter = _begin; iter != _end; iter++ )
        temp.push_back( iter->getRawSource() );
      boost::mutex::scoped_lock( global_lock );
      _begin->getContextCore().makeCurrent();
      alSourceStopv( temp.size(), &temp.front() );
      error::E2E::checkAL();
    }
    template< typename Iterator >
    inline void play( Iterator _begin, Iterator _end,
                     typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, Source > >::type* = 0,
                     typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
      boost::mutex::scoped_lock( global_lock );
      _begin->getContextCore().makeCurrent();
      alSourcePlayv( std::distance( _begin, _end ), _begin );
      error::E2E::checkAL();
    }
    template< typename Iterator >
    inline void pause( Iterator _begin, Iterator _end,
                      typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, Source > >::type* = 0,
                      typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
      std::vector< ALuint > temp;
      Iterator iter;
      for( iter = _begin; iter != _end; iter++ )
        temp.push_back( iter->getRawSource() );
      boost::mutex::scoped_lock( global_lock );
      _begin->getContextCore().makeCurrent();
      alSourcePausev( temp.size(), &temp.front() );
      error::E2E::checkAL();
    }
    template< typename Iterator >
    inline void stop( Iterator _begin, Iterator _end,
                     typename boost::enable_if< boost::is_same< typename std::iterator_traits< Iterator >::value_type, Source > >::type* = 0,
                     typename boost::enable_if< boost::is_pointer< Iterator > >::type* = 0 ) {
      std::vector< ALuint > temp;
      Iterator iter;
      for( iter = _begin; iter != _end; iter++ )
        temp.push_back( iter->getRawSource() );
      boost::mutex::scoped_lock( global_lock );
      _begin->getContextCore().makeCurrent();
      alSourceStopv( temp.size(), &temp.front() );
      error::E2E::checkAL();
    }
  }
}

#endif
