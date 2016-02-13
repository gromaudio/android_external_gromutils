/*
* A2DP sink tester.
*
* This program can be used and distributed without restrictions.
*
* Authors: Ivan Zaitsev
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <media/AudioTrack.h>
#include <media/AudioRecord.h>

using namespace android;

//-----------------------------------------------------------------------------
#define A2DP_SINK_SAMPLERATE    44100
#define AUDIO_IN_BUFF_SIZE      0x200

//-----------------------------------------------------------------------------
sp<AudioTrack>    mAudioTrack;
sp<AudioRecord>   mAudioRecord;
uint8_t           audioBuff[ AUDIO_IN_BUFF_SIZE ];

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  size_t  frame_count;
  ssize_t br;

  (void)argc;
  (void)argv;

  AudioTrack::getMinFrameCount( &frame_count,
                                AUDIO_STREAM_MUSIC,
                                A2DP_SINK_SAMPLERATE );

  mAudioTrack = new AudioTrack( AUDIO_STREAM_MUSIC,
                                A2DP_SINK_SAMPLERATE,
                                AUDIO_FORMAT_PCM_16_BIT,
                                AUDIO_CHANNEL_OUT_STEREO,
                                0,
                                AUDIO_OUTPUT_FLAG_NONE,
                                NULL,
                                NULL,
                                0,
                                0,
                                AudioTrack::TRANSFER_SYNC,
                                NULL,
                                -1 );
  if( mAudioTrack != NULL )
  {
    mAudioTrack->start();
    fprintf( stderr, "AudioTrack OK.\n" );
  }


  mAudioRecord = new AudioRecord( AUDIO_SOURCE_DEFAULT,
                                  A2DP_SINK_SAMPLERATE,
                                  AUDIO_FORMAT_PCM_16_BIT,
                                  AUDIO_CHANNEL_IN_STEREO,
                                  0 );
  if( mAudioRecord != NULL )
  {
    mAudioRecord->start();
    fprintf( stderr, "AudioRecord OK.\n" );
  }


  if( mAudioTrack != NULL && mAudioRecord != NULL )
    while( true )
    {
      br = mAudioRecord->read( audioBuff, AUDIO_IN_BUFF_SIZE );
      mAudioTrack->write( audioBuff, br );
      usleep( 1000 );
    }

  exit(EXIT_SUCCESS);
}
