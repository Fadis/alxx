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

#ifndef OPENALXX_STREAM_HPP
#define OPENALXX_STREAM_HPP

#include <AL/al.h>
#include <AL/alc.h>
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <al++/singleton.hpp>
#include <al++/repack.hpp>
#include <al++/taskremapper.hpp>
#include <al++/error.hpp>
#include <al++/context.hpp>
#include <al++/buffer.hpp>
#include <al++/source.hpp>

namespace al {
  namespace playback {
    extern boost::mutex global_lock;
    
    class StreamCoreCore : public ContextBoundResource {
    public:
      StreamCoreCore( Context &_context, unsigned int _buffer_count )
      : ContextBoundResource( _context ), max( _buffer_count ) {
        unsigned int count;
        for( count = 0; count != max; count++ )
          push( Buffer( _context ) );
      }
      StreamCoreCore( StreamCoreCore &_source )
      : ContextBoundResource( _source.getContextCore() ), max( _source.max ) {
        unsigned int count;
        for( count = 0; count != max; count++ )
          push( Buffer( getContextCore() ) );
      }
      virtual ~StreamCoreCore() {
        while( !empty() )
          pop();
      }
      void pop() {
        boost::mutex::scoped_lock( buffers_guard );
        buffers.pop();
      }
      void push( Buffer _buffer ) {
        boost::mutex::scoped_lock( buffers_guard );
        buffers.push( _buffer );
      }
      bool empty() const {
        boost::mutex::scoped_lock( buffers_guard );
        return buffers.empty();
      }
      Buffer &front() {
        boost::mutex::scoped_lock( buffers_guard );
        return buffers.front();
      }
    private:
      unsigned int max;
      std::queue< Buffer > buffers;
      boost::mutex buffers_guard;
    };
    
    class StreamCore : public StreamCoreCore {
    public:
      template< typename FeederFunction >
      StreamCore( Context &_context, const boost::shared_ptr< TaskRemapper > &_trm, FeederFunction _feeder, unsigned int _buffer_count = 10 )
      : StreamCoreCore( _context, _buffer_count ), source( _context ), trm( _trm ) {
        feeder = _feeder;
      }
      StreamCore( StreamCore &_source )
      : StreamCoreCore( _source ), source( _source.source ) {
        feeder = _source.feeder;
      }
      virtual ~StreamCore() {
        try{
          stop();
        }
        catch ( error::ALError &_alerror ) {
          std::cerr << "ALError throwed during deleting the stream : " << _alerror.what() << std::endl;
        }
        catch ( error::ALCError &_alcerror ) {
          std::cerr << "ALCError throwed during deleting the stream : " << _alcerror.what() << std::endl;
        }
      }
      void play() {
        ALint status = source.getState();
        if( status != AL_PLAYING ) {
          while( !empty() ) {
            feeder( front() );
            source.queueBuffer( front() );
            status = source.getState();
            if( status != AL_PLAYING )
              source.play();
            pop();
          }
          token = trm->insertLoop( boost::bind( &StreamCore::run, this ) );
        }
      }
      void pause() {
        token.reset();
        ALint status = source.getState();
        if( status == AL_PLAYING ) {
          source.pause();
        }
        while( source.getProcessedCount() ) {
          push( source.dequeueBuffer() );
        }
      }
      void stop() {
        token.reset();
        ALint status = source.getState();
        if( status == AL_PLAYING || status == AL_PAUSED ) {
          source.stop();
        }
        while( source.getProcessedCount() ) {
          push( source.dequeueBuffer() );
        }
      }
      void run() {
          ALint status = source.getState();
          if( status == AL_PLAYING ) {
            ALint count = source.getProcessedCount();
            if( count ) {
              Buffer temp_buffer = source.dequeueBuffer();
              feeder( temp_buffer );
              source.queueBuffer( temp_buffer );
            }
          }
          else {
            while( source.getProcessedCount() ) {
              push( source.dequeueBuffer() );
            }
            while( !empty() ) {
              feeder( front() );
              source.queueBuffer( front() );
              status = source.getState();
              if( status != AL_PLAYING )
                source.play();
              pop();
            }
          }
      }
      void wait() {
        source.wait();
      }
    private:
      Source source;
      boost::shared_ptr< TaskRemapper > trm;
      boost::function< void ( Buffer& ) > feeder;
      TaskRemapper::Token token;
    };
    class Stream {
    public:
      template< typename FeederFunction >
      Stream( Context &_context, const boost::shared_ptr< TaskRemapper > &_trm, FeederFunction _feeder, unsigned int _buffer_count = 10 )
      : core( new StreamCore( _context, _trm, _feeder, _buffer_count ) ) {
      }
      Stream( const Stream &_source )
      : core( new StreamCore( *_source.getCore() ) ) {
      }
      void play() {
        core->play();
      }
      void pause() {
        core->pause();
      }
      void stop() {
        core->stop();
      }
      const boost::shared_ptr< StreamCore > &getCore() const {
        return core;
      }
      void wait() {
        core->wait();
      }
    private:
      boost::shared_ptr< StreamCore > core;
    };
  }
}

#endif
