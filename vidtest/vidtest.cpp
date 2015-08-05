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
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <linux/fb.h>
#include <g2d.h>
#include <binder/IServiceManager.h>
#include <binder/ProcessState.h>
#include <media/ICrypto.h>
#include <media/stagefright/foundation/ABuffer.h>
#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/foundation/ALooper.h>
#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/foundation/AString.h>
#include <media/stagefright/DataSource.h>
#include <media/stagefright/MediaCodec.h>
#include <media/stagefright/MediaCodecList.h>
#include <media/stagefright/MediaDefs.h>
#include <media/stagefright/MetaData.h>
#include <media/stagefright/NativeWindowWrapper.h>
#include <gui/ISurfaceComposer.h>
#include <gui/SurfaceComposerClient.h>
#include <gui/Surface.h>
#include <ui/DisplayInfo.h>
#include <android/native_window.h>

using namespace android;

//--------------------------------------------------------------------------
#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define MAX_CAMERA_BUFFERS  4

//--------------------------------------------------------------------------
int                       fd = -1;
bool                      saveRawdata;
bool                      displayData;
const char*               cam_name = NULL;
unsigned int              n_buffers    = 0;
unsigned int              sec_capture  = 2;
unsigned int              capture_mode = 0;
struct v4l2_format        fmt;
struct g2d_buf*           aBuffers[ MAX_CAMERA_BUFFERS ];
sp<SurfaceComposerClient> gComposerClient;
sp<SurfaceControl>        gControl;
sp<Surface>               gSurface;
sp<NativeWindowWrapper>   gNativeWindowWrapper;
struct ANativeWindow*     gNativeWindow;

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
  int   fd;
  char  fileName[ 30 ];

  sprintf( fileName, "/boot/hdmi/frame_%03d.raw", iFrameNum );
  fd = open( fileName, O_CREAT | O_WRONLY, S_IRUSR );
  if( fd != -1 )
  {
    fprintf( stderr, "File write: %s\n", fileName );
    write( fd, buff, buffSize );
  }
  else
  {
    fprintf( stderr, "File create error: %s\n", fileName );
  }
  close( fd );

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
static void process_image( int idx, unsigned long addr )
{
  status_t             res;
  ANativeWindow_Buffer buffer;
  void                *g2dHandle;
  struct g2d_surface   src,
                       dst;

  if( g2d_open( &g2dHandle ) == -1 || g2dHandle == NULL )
  {
    fprintf( stderr, "Fail to open g2d device!\n" );
    return;
  }

  if( displayData )
  {
    int offset = 100 * fmt.fmt.pix.height + 400;

    fprintf( stderr, "D: %02X %02X %02X %02X %02X %02X %02X %02X\n",
    ((char*)aBuffers[idx]->buf_vaddr)[offset + 0],
    ((char*)aBuffers[idx]->buf_vaddr)[offset + 1],
    ((char*)aBuffers[idx]->buf_vaddr)[offset + 2],
    ((char*)aBuffers[idx]->buf_vaddr)[offset + 3],
    ((char*)aBuffers[idx]->buf_vaddr)[offset + 4],
    ((char*)aBuffers[idx]->buf_vaddr)[offset + 5],
    ((char*)aBuffers[idx]->buf_vaddr)[offset + 6],
    ((char*)aBuffers[idx]->buf_vaddr)[offset + 7] );
  }

  if(saveRawdata)
    save_image( (uint8_t*)aBuffers[idx]->buf_vaddr, aBuffers[idx]->buf_size );

  gSurface->lock( &buffer, NULL );

  src.format    = G2D_YUYV;
  src.planes[0] = aBuffers[idx]->buf_paddr;
  src.left      = 0;
  src.top       = 0;
  src.right     = fmt.fmt.pix.width;
  src.bottom    = fmt.fmt.pix.height;
  src.stride    = fmt.fmt.pix.width;
  src.width     = fmt.fmt.pix.width;
  src.height    = fmt.fmt.pix.height;
  src.rot       = G2D_ROTATION_0;

  dst.planes[0] = get_phy_address(buffer.bits);
  dst.left      = 0;
  dst.top       = 0;
  dst.right     = buffer.width;
  dst.bottom    = buffer.height;
  dst.stride    = buffer.stride;
  dst.width     = buffer.width;
  dst.height    = buffer.height;
  dst.rot       = G2D_ROTATION_0;
  dst.format    = G2D_RGB565;

  g2d_blit( g2dHandle, &src, &dst );
  g2d_finish( g2dHandle );

  gSurface->unlockAndPost();

  fprintf( stderr, "." );
}

//--------------------------------------------------------------------------
static int read_frame( void )
{
  struct v4l2_buffer buf;
  unsigned int i, j, cs, err;

  CLEAR( buf );
  buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;

  if( -1 == xioctl( fd, VIDIOC_DQBUF, &buf ) )
  {
    if( errno == EAGAIN )
      return 0;
    else
      errno_exit( "VIDIOC_DQBUF" );
  }

  for( i = 0; i < n_buffers; ++i )
    if( buf.m.offset == (unsigned int)aBuffers[i]->buf_paddr )
      break;

  assert(i < n_buffers);
  process_image( i, buf.m.userptr );

  if( -1 == xioctl( fd, VIDIOC_QBUF, &buf ) )
    errno_exit( "VIDIOC_QBUF" );

  return 1;
}

//--------------------------------------------------------------------------
static void run( void )
{
  unsigned int    count,
                  frames;
  struct timeval  tv1,
                  tv2;

  gettimeofday( &tv1, NULL );

  frames = 30 * sec_capture;
  while( frames-- > 0 )
  {
    for(;;)
    {
      usleep(5000);
      if( read_frame() )
        break;
    }
  }
  gettimeofday( &tv2, NULL );
  fprintf( stderr, "\nCapturing time %3.1fs.\n",( tv2.tv_sec - tv1.tv_sec + ((float)(tv2.tv_usec-tv1.tv_usec))/1000000) );
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
    buf.memory    = V4L2_MEMORY_USERPTR;
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
    g2d_free( aBuffers[ i ] );
  }
}

//--------------------------------------------------------------------------
static void init_mmap( unsigned int buffersize )
{
  struct v4l2_requestbuffers req;

  CLEAR( req );
  req.count   = MAX_CAMERA_BUFFERS;
  req.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory  = V4L2_MEMORY_USERPTR;

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

    aBuffers[ n_buffers ] = g2d_alloc( buffersize, 0 );

    fprintf( stderr, "Alloc buff %d, paddr 0x%08X, size %d\n",
             n_buffers,
             (unsigned int)aBuffers[ n_buffers ]->buf_paddr,
             (unsigned int)aBuffers[ n_buffers ]->buf_size );
    if( !aBuffers[ n_buffers ] )
      fprintf( stderr, "Fail to allocate physical memory for image buffer!\n" );

    memset( &buf, 0, sizeof( buf ) );
    buf.index    = n_buffers;
    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory   = V4L2_MEMORY_USERPTR;
    buf.m.offset = aBuffers[n_buffers]->buf_paddr;
    buf.length   = aBuffers[n_buffers]->buf_size ;

    if( -1 == ioctl(fd, VIDIOC_QUERYBUF, &buf ) )
      fprintf( stderr, "Fail to register image buffer!\n" );
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
  struct v4l2_fmtdesc         fmt_desc;
  struct v4l2_frmsizeenum     frmsize;
  struct v4l2_frmivalenum     frmival;
  struct v4l2_dbg_chip_ident  vid_chip;

  if( ioctl( fd, VIDIOC_DBG_G_CHIP_IDENT, &vid_chip ) >= 0 )
    fprintf( stderr, "Sensor chip name: %s\n", vid_chip.match.name );

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

      frmsize.index++;
    }
    fmt_desc.index++;
  }
  }

  input = 1;
  fprintf( stderr, "Select V4L2 input %d\n", input );
  if( ioctl( fd, VIDIOC_S_INPUT, &input ) < 0 )
    errno_exit("VIDIOC_S_INPUT");

  CLEAR( param );
  param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  param.parm.capture.timeperframe.numerator   = 1;
  param.parm.capture.timeperframe.denominator = 60;
  param.parm.capture.capturemode              = capture_mode;
  if( -1 == xioctl( fd, VIDIOC_S_PARM, &param ) )
    errno_exit( "VIDIOC_S_PARAM" );

  CLEAR( fmt );
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
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

  native_window_set_buffers_format( gNativeWindow, HAL_PIXEL_FORMAT_RGB_565 );
  native_window_set_buffers_dimensions( gNativeWindow, fmt.fmt.pix.width, fmt.fmt.pix.height );
  native_window_set_scaling_mode( gNativeWindow, NATIVE_WINDOW_SCALING_MODE_SCALE_TO_WINDOW );

  init_mmap( fmt.fmt.pix.sizeimage );
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

//--------------------------------------------------------------------------
static void usage( FILE * fp,int argc,char ** argv )
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
  start_capturing();
  run();
  stop_capturing();
  uninit_device();
  close_device();

  return 0;
}


