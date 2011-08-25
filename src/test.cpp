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

#include <al++/al++.hpp>
#include "epiano.h"
#include <unistd.h>
#include <iostream>
#include <cmath>

ALfloat position[ 3 ] = { 1.0f, 0.0f, 0.0f };
ALfloat lat[ 3 ] = { 0.0f, 0.0f, 0.0f };
ALfloat lup[ 3 ] = { 0.0f, 0.0f, 1.0f };


class Feeder {
  public:
  Feeder() : ratio( 1.0f ) {}
  void operator()( al::playback::Buffer &_buffer ) {
    std::vector< ALshort > temp;
    temp.resize( 109 * 10 );
    std::vector< ALshort >::iterator iter;
    float pos;
    for( iter = temp.begin(), pos = 0.0f; iter != temp.end(); iter++, pos += 2.0f * M_PI / 109.0f ) {
      *iter = sinf( pos ) * 32767.0f * ratio;
    }
    ratio *= 0.9f;
    _buffer.setData( temp.begin(), temp.end(), 48000 );
  }
  private:
    float ratio;
};

int main() {
  using namespace al::playback;
  DeviceList device_list;
  Device device( device_list.front() );
  ContextAttribute attr;
  attr.setFrequency( 48000 );
  Context context( device, attr );
  Source source( context );
  Listener listener( context );
  listener.setOrientation( lat, lat + 3, lup, lup + 3 );
  source.setPosition( position, position + 3 );
   Buffer buffer( context );
    Buffer buffer2 = buffer;
    buffer.setData( epiano, epiano + 76833, 48000 );
    buffer2.setData( epiano, epiano + 76833, 48000 );
    source.queueBuffer( buffer );
    source.queueBuffer( buffer2 );
    source.play();
  Source source2( source );
  {
    ALCExtensionList alcel( device );
    ALCExtensionList::const_iterator iter;
    std::cout << "= ALC Extensions =" << std::endl;
    for( iter = alcel.begin(); iter != alcel.end(); iter++ )
      std::cout << *iter << std::endl;
  }
  {
    ALExtensionList alel( context );
    ALExtensionList::const_iterator iter;
    std::cout << "= AL Extensions =" << std::endl;
    for( iter = alel.begin(); iter != alel.end(); iter++ )
      std::cout << *iter << std::endl;
  }
  sleep( 3 );
  boost::shared_ptr< al::TaskRemapper > trm( new al::TaskRemapper );
  Stream stream1( context, trm, Feeder(),  10 );
  Stream stream2( context, trm, Feeder(),  10 );
  Stream stream3( context, trm, Feeder(),  10 );
  Stream stream4( context, trm, Feeder(),  10 );
  Stream stream5( context, trm, Feeder(),  10 );
  Stream stream6( context, trm, Feeder(),  10 );
  stream1.play();
  sleep( 1 );
  stream2.play();
  sleep( 1 );
  stream3.play();
  sleep( 1 );
  stream4.play();
  sleep( 1 );
  stream5.play();
  sleep( 1 );
  stream6.play();
}
