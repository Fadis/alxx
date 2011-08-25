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

#ifndef OPENALXX_TASKREMAPPER_HPP
#define OPENALXX_TASKREMAPPER_HPP

#include <tbb/parallel_for_each.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/tbb_thread.h>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <vector>


#include <iostream>
namespace al {
  class TaskRemapper {
  public:
    typedef tbb::concurrent_hash_map< void*, boost::shared_ptr< boost::function< void () > > > MapType;
    typedef boost::shared_ptr< void > Token;
    typedef std::vector< Token > Once;
    typedef MapType::accessor Accessor;
    TaskRemapper()
    : ctrl_thread( boost::bind( &TaskRemapper::run, this ) ), map( new MapType ), map_guard( new boost::mutex ) {
    }
    TaskRemapper( const TaskRemapper &_source )
    : ctrl_thread( boost::bind( &TaskRemapper::run, this ) ), map( new MapType ), map_guard( new boost::mutex ) {
    }
    virtual ~TaskRemapper() {
      end_flag = true;
      ctrl_thread.interrupt();
      ctrl_thread.join();
    }
    template< typename Function >
    Token insertLoop( Function _func ) {
      boost::shared_ptr< boost::function< void () > > function( new boost::function< void () > );
      (*function) = _func;
      void *key = static_cast<void*>( function.get() );
      
      Accessor accessor;
      
      {
        boost::mutex::scoped_lock lock( *map_guard );
        map->insert( accessor, key );
        accessor->second = function;
      }
      
      Token deleter( key, boost::bind( &TaskRemapper::erase, map, map_guard, _1 ) );
      return deleter;
    }
    template< typename Function >
    void insertOnce( Function _func ) {
      boost::shared_ptr< boost::function< void () > > function( new boost::function< void () > );
      (*function) = _func;
      void *key = static_cast<void*>( function.get() );
      
      Accessor accessor;
      
      {
        boost::mutex::scoped_lock map_lock( *map_guard );
        map->insert( accessor, key );
        accessor->second = function;
        Token deleter( key, boost::bind( &TaskRemapper::eraseNonBlock, map, _1 ) );
        once.push_back( deleter );
      }
    }
  private:
    void run() {
      while( 1 ) {
        {
          boost::this_thread::interruption_point();
          {
            boost::this_thread::disable_interruption di;
            boost::mutex::scoped_lock lock( *map_guard );
            if( !map->empty() ) {
              tbb::parallel_for_each( map->begin(), map->end(), &TaskRemapper::runEach );
              once.clear();
            }
          }
        }
        boost::this_thread::interruption_point();
        tbb::this_tbb_thread::sleep( tbb::tick_count::interval_t( 0.001 ) );
        boost::this_thread::interruption_point();
      }
    }
    
    static void runEach( std::pair<void* const, boost::shared_ptr<boost::function<void ()> > > &_element ) {
      (*_element.second)();
    }
    static void erase( boost::shared_ptr< MapType > _map, boost::shared_ptr< boost::mutex > _map_guard, void* _key ) {
      boost::mutex::scoped_lock lock( *_map_guard );
      _map->erase( _key );
    }
    static void eraseNonBlock( boost::shared_ptr< MapType > _map, void* _key ) {
      _map->erase( _key );
    }
    bool end_flag;
    boost::thread ctrl_thread;
    boost::shared_ptr< boost::mutex > map_guard;
    boost::shared_ptr< MapType > map;
    Once once;
  };
}

#endif
