#include <al++/al++.hpp>
#include "epiano.h"

int main() {
  using namespace al::playback;
  DeviceList device_list;
  Device device( device_list.front() );
  Context context( device );
  Source source( context );
  Buffer buffer( context );
  buffer.setData( epiano, epiano + EPIANO_LENGTH, 48000 );
  source.queueBuffer( buffer );
  source.play();
  source.wait();
}
