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

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>
#include <vector>
#include <string>
#include <queue>
#include <map>
#include <set>
#include <boost/spirit/include/qi.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/type_traits.hpp>
#include <boost/utility/enable_if.hpp>
#include <tbb/parallel_for_each.h>
#include <tbb/blocked_range.h>
#include <iostream>

#include <al++/al++.hpp>

namespace al {
  namespace playback {
    boost::mutex global_lock;
    
    DeviceList::DeviceList() {
      const ALCchar *devices = alcGetString( NULL, ALC_DEVICE_SPECIFIER );
      while( *devices ) {
        push_back( devices );
        for( ; *devices; devices++ );
        devices++;
      }
    }
    ALCdevice *DeviceManager::open( const char *_device_name ) {
      boost::mutex::scoped_lock lock( map_guard );
      InternalContainer::iterator search_result = opened_devices.find( _device_name );
      if( search_result == opened_devices.end() ) {
        ALCdevice* device = alcOpenDevice( _device_name );
        if( !device )
          throw -1;
        opened_devices[ _device_name ].first = device;
        opened_devices[ _device_name ].second++;
        return device;
      }
      else {
        search_result->second.second++;
        return search_result->second.first;
      }
    }
    void DeviceManager::close( ALCdevice *_device, const char *_device_name ) {
      boost::mutex::scoped_lock lock( map_guard );
      InternalContainer::iterator search_result = opened_devices.find( _device_name );
      if( search_result == opened_devices.end() ) {
        throw -1;
      }
      else {
        search_result->second.second--;
        if( !search_result->second.second ) {
          alcCloseDevice( search_result->second.first );
          opened_devices.erase( search_result );
        }
      }
    }    
    Context::Context( Device &_device )
    : device( _device ) {
      ALCcontext *context = alcCreateContext( device.getRawDevice().get(), NULL );
      if( !context )
        error::E2E::checkALC( device.getRawDevice().get() );
      raw_context.reset( context, boost::bind( &Context::deleter, _1, device ) );
    }
    
    Context::Context( Device &_device, const ContextAttribute &_attr )
    : device( _device ), attr( _attr ) {
      ALCint attr_list[ 7 ];
      (*attr).getAttrList( attr_list, attr_list + 7 );
      ALCcontext *context = alcCreateContext( device.getRawDevice().get(), attr_list );
      if( !context )
        error::E2E::checkALC( device.getRawDevice().get() );
      raw_context.reset( context, boost::bind( &Context::deleter, _1, device ) );
    }
    
    Context::Context( const Context &_source )
    : device( _source.device ), attr( _source.attr ) {
      if( attr ) {
        ALCint attr_list[ 7 ];
        (*attr).getAttrList( attr_list, attr_list + 7 );
        ALCcontext *context = alcCreateContext( device.getRawDevice().get(), attr_list );
        if( !context )
          error::E2E::checkALC( device.getRawDevice().get() );
        raw_context.reset( context, boost::bind( &Context::deleter, _1, device ) );
      }
      else {
        ALCcontext *context = alcCreateContext( device.getRawDevice().get(), NULL );
        if( !context )
          error::E2E::checkALC( device.getRawDevice().get() );
        raw_context.reset( context, boost::bind( &Context::deleter, _1, device ) );
      }
    }
    void Context::deleter( ALCcontext *_context, Device _device ) {
      try {
        boost::mutex::scoped_lock( global_lock );
        if( _context == alcGetCurrentContext() ) {
          alcMakeContextCurrent( NULL );
          error::E2E::checkALC( _device.getRawDevice().get() );
        }
        alcDestroyContext( _context );
        error::E2E::checkALC( _device.getRawDevice().get() );
      }
      catch ( error::ALCError &_alcerror ) {
        std::cerr << "ALCError throwed during deleting the context : " << _alcerror.what() << std::endl;
      }
    }
    ALCExtensionList::ALCExtensionList( Device &_device ) {
      boost::mutex::scoped_lock( global_lock );
      const ALCchar *extensions = alcGetString( _device.getRawDevice().get(), ALC_EXTENSIONS );
      error::E2E::checkALC( _device.getRawDevice().get() );
      const ALCchar *end;
      for( end = extensions; *end; end++ );
      using namespace boost::spirit;
      qi::parse( extensions, end, +ascii::graph % ' ', *this );
    }
    
    ALExtensionList::ALExtensionList( Context &_context ) {
      boost::mutex::scoped_lock( global_lock );
      _context.getCore().makeCurrent();
      const ALchar *extensions = alGetString( AL_EXTENSIONS );
      error::E2E::checkAL();
      const ALchar *end;
      for( end = extensions; *end; end++ );
      using namespace boost::spirit;
      qi::parse( extensions, end, +ascii::graph % ' ', *this );
    }
    
    void BufferCore::deleter( void *_dummy, ContextCore &_context_core, ALuint _buffer ) {
      try {
        boost::mutex::scoped_lock( global_lock );
        _context_core.makeCurrent();
        alDeleteBuffers( 1, &_buffer );
        error::E2E::checkAL();
      }
      catch ( error::ALError &_alerror ) {
        std::cerr << "ALError throwed during deleting the buffer : " << _alerror.what() << std::endl;
      }
      catch ( error::ALCError &_alcerror ) {
        std::cerr << "ALCError throwed during deleting the buffer : " << _alcerror.what() << std::endl;
      }
    }
    
    Source::Source( Context &_context )
    : ContextBoundResource( _context ), distance_model( AL_INVERSE_DISTANCE_CLAMPED ) {
      {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGenSources( 1, &source );
        error::E2E::checkAL();
      }
      source_deleter.reset( static_cast<void*>( 0 ), boost::bind( &Source::deleter, _1, getContextCore(), source ) );
    }
    Source::Source( const Source &_source )
    : ContextBoundResource( _source.getContextCore() ) {
      {
        boost::mutex::scoped_lock( global_lock );
        getContextCore().makeCurrent();
        alGenSources( 1, &source );
        error::E2E::checkAL();
      }
      source_deleter.reset( static_cast<void*>( 0 ), boost::bind( &Source::deleter, _1, getContextCore(), source ) );        
      looping( _source.isLooping() );
      listenerRelative( _source.isListenerRelative() );
      {
        ALfloat gain;
        _source.getGain( gain );
        setGain( gain );
      }
      {
        ALfloat pitch;
        _source.getPitch( pitch );
        setPitch( pitch );
      }
      {
        ALfloat position[ 3 ];
        _source.getPosition( position, position + 3 );
        setPosition( position, position + 3 );
      }
      {
        ALfloat velocity[ 3 ];
        _source.getVelocity( velocity, velocity + 3 );
        setVelocity( velocity, velocity + 3 );
      }
      {
        ALfloat direction[ 3 ];
        _source.getDirection( direction, direction + 3 );
        setDirection( direction, direction + 3 );
      }
      {
        ALenum type;
        ALfloat reference, max, rolloff;
        _source.getDistanceModel( type, reference, max, rolloff );
        setDistanceModel( type, reference, max, rolloff );
      }
      {
        ALfloat inner_angle, outer_angle, outer_gain;
        _source.getCone( inner_angle, outer_angle, outer_gain );
        setCone( inner_angle, outer_angle, outer_gain );
      }
      {
        const_iterator iter;
        for( iter = _source.begin(); iter != _source.end(); iter++ ) {
          ALuint raw_buffer = iter->get();
          {
            boost::mutex::scoped_lock( global_lock );
            getContextCore().makeCurrent();
            alSourceQueueBuffers( source, 1, &raw_buffer );
            error::E2E::checkAL();
          }
          push_front( *iter );
        }
      }
      {
        ALint offset;
        _source.getSampleOffset( offset );
        setSampleOffset( offset );
      }
    }
    void Source::deleter( void *_dummy, ContextCore &_context_core, ALuint _source ) {
      try {
        boost::mutex::scoped_lock( global_lock );
        _context_core.makeCurrent();
        alSourceStop( _source );
        error::E2E::checkAL();
        ALint left_buffers;
        alGetSourcei( _source, AL_BUFFERS_PROCESSED, &left_buffers );
        if( left_buffers ) {
          for( ; left_buffers; left_buffers-- ) {
            ALuint raw_buffer;
            alSourceUnqueueBuffers( _source, 1, &raw_buffer );
            error::E2E::checkAL();
          }
        }
        alDeleteSources( 1, &_source );
        error::E2E::checkAL();
      }
      catch ( error::ALError &_alerror ) {
        std::cerr << "ALError throwed during deleting the source : " << _alerror.what() << std::endl;
      }
      catch ( error::ALCError &_alcerror ) {
        std::cerr << "ALCError throwed during deleting the source : " << _alcerror.what() << std::endl;
      }
    }
  }
  namespace record {
  }
}
