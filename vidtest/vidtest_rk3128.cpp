/*
* Framebuffer & V4L2 video capture example.
* Use freescale g2d lib for resize and CSC.
*
* This program can be used and distributed without restrictions.
*
* Authors: Ivan Zaitsev
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <binder/IPCThreadState.h>
#include <binder/ProcessState.h>
#include <media/stagefright/NativeWindowWrapper.h>
#include <gui/ISurfaceComposer.h>
#include <gui/SurfaceComposerClient.h>
#include <gui/Surface.h>
#include <ui/DisplayInfo.h>
#include <android/native_window.h>


using namespace android;


struct cam_buf {
  uint32_t  buf_paddr;
  uint8_t*  buf_vaddr;
  size_t    buf_size;
};

//--------------------------------------------------------------------------
#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define MAX_CAMERA_BUFFERS  2

//--------------------------------------------------------------------------
int                       fd = -1;
int                       memType;
int                       frames;
bool                      saveRawdata;
bool                      displayData;
const char*               cam_name = NULL;
unsigned int              n_buffers    = 0;
unsigned int              sec_capture  = 2;
unsigned int              capture_mode = 0;
struct v4l2_format        fmt;
struct cam_buf*           aBuffers[ MAX_CAMERA_BUFFERS ];
sp<SurfaceComposerClient> gComposerClient;
sp<SurfaceControl>        gControl;
sp<Surface>               gSurface;
sp<NativeWindowWrapper>   gNativeWindowWrapper;
struct ANativeWindow*     gNativeWindow;

//--------------------------------------------------------------------------
static int64_t GetTime()
{
  struct timeval now;
  gettimeofday(&now, NULL);
  return((int64_t)now.tv_sec) * 1000000 + ((int64_t)now.tv_usec);
}

//--------------------------------------------------------------------------
static void errno_exit( const char * s )
{
  fprintf( stderr, "%s error %d, %s\n",s, errno, strerror( errno ) );
  exit( EXIT_FAILURE );
}

//--------------------------------------------------------------------------
static int xioctl( int fd,int request,void * arg )
{
  int r;
  do{
    r = ioctl( fd, request, arg );
  }while( -1 == r && EINTR == errno );
  return r;
}

//--------------------------------------------------------------------------
inline int clip( int value, int min, int max )
{
  return (value > max ? max : value < min ? min : value);
}

//--------------------------------------------------------------------------
static void save_image( uint8_t* buff, ssize_t buffSize )
{
  static int iFrameNum = 0;
  int   fd_raw;
  char  fileName[ 50 ];

  sprintf( fileName, "/mnt/usb_storage/frame_%03d.raw", iFrameNum );
  fd_raw = open( fileName, O_CREAT | O_WRONLY, S_IRUSR );
  if( fd_raw != -1 )
  {
    fprintf( stderr, "File write: %s\n", fileName );
    write( fd_raw, buff, buffSize );
  }
  else
  {
    fprintf( stderr, "File create error: %s\n", fileName );
  }
  close( fd_raw );

  iFrameNum++;
}

// --------------------------------------------------------------------------------
#define PAGEMAP_LENGTH 8
#define PAGE_SHIFT 12
uint32_t get_phy_address(void *addr)
{
   // Open the pagemap file for the current process
   FILE *pagemap = fopen("/proc/self/pagemap", "rb");

   // Seek to the page that the buffer is on
   uint32_t offset = (uint32_t)addr / getpagesize() * PAGEMAP_LENGTH;
   if(fseek(pagemap, offset, SEEK_SET) != 0) {
      fprintf(stderr, "Failed to seek pagemap to proper location\n");
      exit(1);
   }

   // The page frame number is in bits 0-54 so read the first 7 bytes and clear the 55th bit
   uint64_t page_frame_number = 0;
   fread(&page_frame_number, 1, PAGEMAP_LENGTH-1, pagemap);

   page_frame_number &= 0x7FFFFFFFFFFFFF;

   fclose(pagemap);



   return ( page_frame_number << PAGE_SHIFT ) + ((uint32_t)addr % getpagesize());
}

//--------------------------------------------------------------------------
static void process_image( int idx, uint8_t* buff, size_t size )
{
  ANativeWindow_Buffer buffer;
  size_t          y_size  = size / 2;
  size_t          uv_size = size / 4;
  uint64_t        start_time;

  if( displayData )
  {
    int offset = 100 * fmt.fmt.pix.height;

    fprintf( stderr, "D: %02X %02X %02X %02X %02X %02X %02X %02X\n",
    buff[ offset + 0 ],
    buff[ offset + 1 ],
    buff[ offset + 2 ],
    buff[ offset + 3 ],
    buff[ offset + 4 ],
    buff[ offset + 5 ],
    buff[ offset + 6 ],
    buff[ offset + 7 ] );
  }

  if( saveRawdata )
    save_image( buff, size );

  gSurface->lock( &buffer, NULL );

  start_time = GetTime();
  memcpy( buffer.bits, buff, y_size + uv_size );
//  fprintf( stderr, "f: %lld\n", GetTime() - start_time );

  gSurface->unlockAndPost();
}

}
//--------------------------------------------------------------------------
static int read_frame( void )
{
  struct v4l2_buffer buf;
  unsigned int i, j, cs, err;

  CLEAR( buf );
  buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = memType;

  if( -1 == xioctl( fd, VIDIOC_DQBUF, &buf ) )
  {
    if( errno == EAGAIN )
    {
      fprintf( stderr, "!\n" );
      return 0;
    }
    else
      errno_exit( "VIDIOC_DQBUF" );
  }

  fprintf( stderr, "." );

  process_image( buf.index,
                (uint8_t*)aBuffers[ buf.index ]->buf_vaddr,
                aBuffers[ buf.index ]->buf_size );

  if( -1 == xioctl( fd, VIDIOC_QBUF, &buf ) )
    errno_exit( "VIDIOC_QBUF" );

  return 1;
}

//--------------------------------------------------------------------------
static void run( void )
{
  struct timeval  tv1,
                  tv2;
  float           time_elapsed;

  gettimeofday( &tv1, NULL );
  gettimeofday( &tv2, NULL );

  frames       = 0;
  time_elapsed = 0;
  while( time_elapsed < sec_capture )
  {
    if( read_frame() )
    {
      frames++;
    }

    gettimeofday( &tv2, NULL );
    time_elapsed = tv2.tv_sec - tv1.tv_sec + ((float)(tv2.tv_usec-tv1.tv_usec))/1000000;
  }

  fprintf( stderr, "\nCapturing %d frames in %3.1fs = %3.1ffps\n", frames, time_elapsed, frames / time_elapsed );
}

//--------------------------------------------------------------------------
static void stop_capturing( void )
{
  enum v4l2_buf_type type;

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if( -1 == xioctl( fd, VIDIOC_STREAMOFF, &type ) )
    errno_exit( "VIDIOC_STREAMOFF" );
}

//--------------------------------------------------------------------------
static void start_capturing( void )
{
  unsigned int i;
  enum v4l2_buf_type type;

  for( i = 0; i < n_buffers; ++i )
  {
    struct v4l2_buffer buf;

    CLEAR( buf );
    buf.type      = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory    = memType;
    buf.index     = i;
    buf.m.offset  = aBuffers[i]->buf_paddr;

    fprintf( stderr,"Queue buff %d, paddr 0x%08X\n",
             buf.index,
             buf.m.offset );

    if( -1 == xioctl(fd, VIDIOC_QBUF, &buf ) )
      fprintf( stderr, "VIDIOC_QBUF error\n" );
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if( -1 == xioctl( fd, VIDIOC_STREAMON, &type ) )
    errno_exit( "VIDIOC_STREAMON" );
}

//--------------------------------------------------------------------------
static void uninit_device( void )
{
  for( size_t i = 0; i < n_buffers; ++i )
  {
    munmap( aBuffers[ n_buffers ]->buf_vaddr, aBuffers[ n_buffers ]->buf_size );
//    free( aBuffers[ n_buffers ] );
  }
}

//--------------------------------------------------------------------------
static void init_buffers_mmap( unsigned int buffersize )
{
  struct v4l2_requestbuffers req;

  CLEAR( req );
  req.count   = MAX_CAMERA_BUFFERS;
  req.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory  = V4L2_MEMORY_MMAP;

  if( -1 == xioctl( fd, VIDIOC_REQBUFS, &req ) )
  {
    if( EINVAL == errno )
    {
      fprintf( stderr, "%s does not support memory mapping\n", cam_name );
      exit( EXIT_FAILURE );
    }else{
      errno_exit( "VIDIOC_REQBUFS" );
    }
  }

  if( req.count < MAX_CAMERA_BUFFERS )
  {
    fprintf( stderr, "Insufficient buffer memory on %s\n", cam_name );
    exit( EXIT_FAILURE );
  }

  for( n_buffers = 0; n_buffers < req.count; ++n_buffers )
  {
    struct v4l2_buffer buf;

    CLEAR(buf);
    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;

    if( -1 == ioctl( fd, VIDIOC_QUERYBUF, &buf ) )
      fprintf( stderr, "Fail to query image buffer!\n" );

    aBuffers[ n_buffers ] = (struct cam_buf*)malloc( sizeof( struct cam_buf ) );
    aBuffers[ n_buffers ]->buf_size   = buf.length;
    aBuffers[ n_buffers ]->buf_paddr  = buf.m.offset;
    aBuffers[ n_buffers ]->buf_vaddr  = (uint8_t*)mmap( NULL,
                                                        buf.length,
                                                        PROT_READ | PROT_WRITE,
                                                        MAP_SHARED,
                                                        fd,
                                                        buf.m.offset );

    if( MAP_FAILED == aBuffers[ n_buffers ]->buf_vaddr )
      errno_exit( "mmap" );

    fprintf( stderr, "Mmap buff %d, paddr 0x08%x 0x08%x, vaddr 0x08%x, size %d\n",
             n_buffers,
             (unsigned int)aBuffers[ n_buffers ]->buf_paddr,
             get_phy_address(aBuffers[ n_buffers ]->buf_vaddr),
             (unsigned int)aBuffers[ n_buffers ]->buf_vaddr,
             (unsigned int)aBuffers[ n_buffers ]->buf_size );
  }
}

//--------------------------------------------------------------------------
static void init_device( void )
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_streamparm param;
  unsigned int min;
  int input;

  if(-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap))
  {
    if(EINVAL == errno)
    {
      fprintf( stderr, "%s is no V4L2 device\n", cam_name );
      exit( EXIT_FAILURE );
    }else{
      errno_exit( "VIDIOC_QUERYCAP" );
    }
  }

  if( !( cap.capabilities & V4L2_CAP_VIDEO_CAPTURE ) )
  {
    fprintf( stderr, "%s is no video capture device\n", cam_name );
    exit( EXIT_FAILURE );
  }

  if( !( cap.capabilities & V4L2_CAP_STREAMING ) )
  {
    fprintf (stderr, "%s does not support streaming i/o\n",cam_name);
    exit( EXIT_FAILURE );
  }

  // Scan for supported formats.
  {
  struct v4l2_input           input;
  struct v4l2_fmtdesc         fmt_desc;
  struct v4l2_frmsizeenum     frmsize;
  struct v4l2_frmivalenum     frmival;
  unsigned int                fmt_index;


  CLEAR( input );
  input.index = 0;
  while( ioctl( fd, VIDIOC_ENUMINPUT, &input ) == 0 )
  {
    fprintf( stderr, "Input %d: name=%s audio_samplerate=%d\n", input.index, input.name, input.audioset );
    input.index++;
  }

  CLEAR( fmt );
  fmt_index      = 0;
  fmt_desc.index = 0;
  fmt_desc.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while( ioctl( fd, VIDIOC_ENUM_FMT, &fmt_desc ) == 0 )
  {
    frmsize.pixel_format = fmt_desc.pixelformat;
    frmsize.index = 0;
    fprintf( stderr, "Resolutions for %X \"%s\":\n", fmt_desc.pixelformat, fmt_desc.description );
    while( ioctl( fd, VIDIOC_ENUM_FRAMESIZES, &frmsize ) == 0 )
    {
      fprintf( stderr, "\t%dx%d\n",
               frmsize.discrete.width,
               frmsize.discrete.height );
      if( fmt_index == capture_mode )
      {
        fmt.fmt.pix.pixelformat = fmt_desc.pixelformat;
        fmt.fmt.pix.width       = frmsize.discrete.width;
        fmt.fmt.pix.height      = frmsize.discrete.height;
      }
      frmsize.index++;
      fmt_index++;
    }
    fmt_desc.index++;
  }
  }

  input = 1;
  fprintf( stderr, "Select V4L2 input %d\n", input );
  if( ioctl( fd, VIDIOC_S_INPUT, &input ) < 0 )
    fprintf( stderr, "VIDIOC_S_INPUT not supported\n");

  CLEAR( param );
  param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  param.parm.capture.timeperframe.numerator   = 1;
  param.parm.capture.timeperframe.denominator = 60;
  param.parm.capture.capturemode              = capture_mode;
  if( -1 == xioctl( fd, VIDIOC_S_PARM, &param ) )
    errno_exit( "VIDIOC_S_PARAM" );


  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.field       = V4L2_FIELD_NONE;

  if( -1 == xioctl( fd, VIDIOC_S_FMT, &fmt ) )
    errno_exit( "VIDIOC_S_FMT" );

  fprintf( stderr, "%dx%d, %c%c%c%c, %d, %d\n",
           fmt.fmt.pix.width,
           fmt.fmt.pix.height,
           (   fmt.fmt.pix.pixelformat & 0xFF ),
           ( ( fmt.fmt.pix.pixelformat >> 8 ) & 0xFF ),
           ( ( fmt.fmt.pix.pixelformat >> 16 ) & 0xFF ),
           ( ( fmt.fmt.pix.pixelformat >> 24 ) & 0xFF ),
           fmt.fmt.pix.field,
           fmt.fmt.pix.sizeimage );

  native_window_set_buffers_format( gNativeWindow, HAL_PIXEL_FORMAT_YCrCb_420_SP );
  native_window_set_buffers_dimensions( gNativeWindow, fmt.fmt.pix.width, fmt.fmt.pix.height );
  native_window_set_scaling_mode( gNativeWindow, NATIVE_WINDOW_SCALING_MODE_SCALE_TO_WINDOW );

  init_buffers_mmap( fmt.fmt.pix.sizeimage );
}

//--------------------------------------------------------------------------
static void close_device( void )
{
  close( fd );
}

//--------------------------------------------------------------------------
static void open_device( void )
{
  struct stat st;

  if( -1 == stat( cam_name, &st ) )
  {
    fprintf( stderr, "Cannot identify '%s': %d, %s\n", cam_name, errno, strerror( errno ) );
    exit( EXIT_FAILURE );
  }

  if( !S_ISCHR( st.st_mode ) )
  {
    fprintf( stderr, "%s is no device\n", cam_name );
    exit( EXIT_FAILURE );
  }

  //open camera
  fd = open( cam_name, O_RDWR| O_NONBLOCK, 0 );
  if( -1 == fd )
  {
    fprintf( stderr, "Cannot open '%s': %d, %s\n",cam_name, errno, strerror( errno ) );
    exit( EXIT_FAILURE );
  }
}

// --------------------------------------------------------------------------------
static void initOutputSurface( void )
{
  // set up the thread-pool
  sp<ProcessState> proc(ProcessState::self());
  ProcessState::self()->startThreadPool();

  gComposerClient = new SurfaceComposerClient;
  if( OK == gComposerClient->initCheck() )
  {
    DisplayInfo info;
    sp<IBinder> display( SurfaceComposerClient::getBuiltInDisplay(
                         ISurfaceComposer::eDisplayIdMain ) );

    SurfaceComposerClient::getDisplayInfo( display, &info );
    size_t displayWidth  = info.w;
    size_t displayHeight = info.h;

    fprintf( stderr, "display is %d x %d\n", displayWidth, displayHeight );

    gControl = gComposerClient->createSurface( String8("A Surface"),
                                               displayWidth,
                                               displayHeight,
                                               PIXEL_FORMAT_RGB_565,
                                               0 );

    if( ( gControl != NULL ) && ( gControl->isValid() ) )
    {
      SurfaceComposerClient::openGlobalTransaction();
      if( ( OK == gControl->setLayer( INT_MAX ) ) &&
          ( OK == gControl->show() ) )
      {
        SurfaceComposerClient::closeGlobalTransaction();
        gSurface = gControl->getSurface();
      }
    }
  }

  if( gSurface == NULL )
  {
    fprintf( stderr, "Screen surface create error\n" );
  }
  gNativeWindowWrapper = new NativeWindowWrapper( gSurface );
  gNativeWindow = gSurface.get();

  fprintf( stderr, "Screen surface created\n" );
  fprintf( stderr, "Screen surface %s\n", (gSurface->isValid(gSurface)?"valid":"invalid") );
}

void closeOutputSurface(void)
{

}

//--------------------------------------------------------------------------
static void usage( FILE * fp,int argc, char ** argv )
{
  fprintf( fp,
           "Usage: %s [options]\n\n"
           "Options:\n"
           "-c | --cam-device Camera device name [/dev/video]\n"
           "-t | --how long will display in seconds\n"
           "-m | --video mode to activate [0]\n"
           "-s | --save raw pictures to /boot/hdmi folder\n"
           "-h | --help Print this message\n"
           "-d | --display raw data\n"
           "",
           argv[0] );
}

//--------------------------------------------------------------------------
static const char short_options[] = "c:ht:m:sd";
static const struct option long_options[] =
{
  { "cam-device", required_argument, NULL, 'c' },
  { "help",       no_argument,       NULL, 'h' },
  { "time",       no_argument,       NULL, 't' },
  { "mode",       no_argument,       NULL, 'm' },
  { "save",       no_argument,       NULL, 's' },
  { "display",    no_argument,       NULL, 'd' },
  { 0, 0, 0, 0 }
};

//--------------------------------------------------------------------------
int main( int argc, char ** argv )
{
  cam_name = "/dev/video0";
  saveRawdata = false;
  displayData = false;
  memType     = V4L2_MEMORY_MMAP;

  for(;;)
  {
    int index;
    int c;

    c = getopt_long( argc, argv, short_options, long_options, &index );
    if( -1 == c )
      break;

    switch( c )
    {
      case 0:
        break;

      case 'c':
        cam_name = optarg;
        break;

      case 'h':
        usage( stdout, argc, argv );
        exit( EXIT_SUCCESS );
        break;

      case 't':
        sec_capture = atoi( optarg );
        break;

      case 'm':
        capture_mode = atoi( optarg );
        break;

      case 's':
        saveRawdata = true;
        break;

      case 'd':
        displayData = true;
        break;

      default:
        usage( stderr, argc, argv );
        exit( EXIT_FAILURE );
        break;
    }
  }

  initOutputSurface();
  open_device();
  init_device();
  sleep(1);
//  n_buffers = 1;
  start_capturing();
  run();
  stop_capturing();
  uninit_device();
  close_device();
  closeOutputSurface();

  return 0;
}


