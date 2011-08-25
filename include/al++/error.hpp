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

#ifndef OPENALXX_ERROR_HPP
#define OPENALXX_ERROR_HPP

#include <AL/al.h>
#include <AL/alc.h>
#include <stdexcept>

namespace al {
  namespace error {
    class ALCError : public std::domain_error {
    public:
      ALCError( const char *_message ) : std::domain_error( _message ) {
      }
    };
    class ALCInvalidDevice : public ALCError {
    public:
      ALCInvalidDevice( const char *_message = "Invalid Device" ) : ALCError( _message ) {
      }
    };
    class ALCInvalidContext : public ALCError {
    public:
      ALCInvalidContext( const char *_message = "Invalid Context" ) : ALCError( _message ) {
      }
    };
    class ALCInvalidEnum : public ALCError {
    public:
      ALCInvalidEnum( const char *_message = "Invalid Enum" ) : ALCError( _message ) {
        
      }
    };
    class ALCInvalidValue : public ALCError {
    public:
      ALCInvalidValue( const char *_message = "Invalid Value" ) : ALCError( _message ) {
        
      }
    };
    class ALCOutOfMemory : public ALCError {
    public:
      ALCOutOfMemory( const char *_message = "Out Of Memory" ) : ALCError( _message ) {
        
      }
    };
    class ALError : public std::domain_error {
    public:
      ALError( const char *_message ) : std::domain_error( _message ) {
        
      }
    };
    class ALInvalidName : public ALError {
    public:
      ALInvalidName( const char *_message = "Invalid Name" ) : ALError( _message ) {
        
      }
    };
    class ALInvalidEnum : public ALError {
    public:
      ALInvalidEnum( const char *_message = "Invalid Enum" ) : ALError( _message ) {
        
      }
    };
    class ALInvalidValue : public ALError {
    public:
      ALInvalidValue( const char *_message = "Invalid Value" ) : ALError( _message ) {
        
      }
    };
    class ALInvalidOperation : public ALError {
    public:
      ALInvalidOperation( const char *_message = "Invalid Operation" ) : ALError( _message ) {
        
      }
    };
    class ALOutOfMemory : public ALError {
    public:
      ALOutOfMemory( const char *_message = "Out Of Memory" ) : ALError( _message ) {
        
      }
    };
    
    class ExtensionNotAvailable : public std::domain_error {
    public:
      ExtensionNotAvailable( const char *_message = "Extension  Not Available" ) : std::domain_error( _message ) {
      }
    };
    
    class E2E {
    public:
      inline static void checkALC( ALCdevice *_device ) {
        switch( alcGetError( _device ) ) {
          case ALC_INVALID_DEVICE:
            throw ALCInvalidDevice();
          case ALC_INVALID_CONTEXT:
            throw ALCInvalidContext();
          case ALC_INVALID_ENUM:
            throw ALCInvalidEnum();
          case ALC_INVALID_VALUE:
            throw ALCInvalidValue();
          case ALC_OUT_OF_MEMORY:
            throw ALCOutOfMemory();
          default:
            return;
        }
      }
      inline static void checkAL() {
        switch( alGetError() ) {
          case AL_INVALID_NAME:
            throw ALInvalidName();
          case AL_INVALID_ENUM:
            throw ALInvalidEnum();
          case AL_INVALID_VALUE:
            throw ALInvalidValue();
          case AL_INVALID_OPERATION:
            throw ALInvalidOperation();
          case AL_OUT_OF_MEMORY:
            throw ALOutOfMemory();
          default:
            return;
        }
      }
    };
  }
}

#endif
