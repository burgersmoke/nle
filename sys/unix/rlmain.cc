#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <termios.h>

#ifndef WIN32
    // does not exist under Windows
    #include <unistd.h>
#endif

#ifdef WIN32
    // #include <io.h>

    #include <termiWin.h>

    // and define this for Windows
    // https://stackoverflow.com/a/13531812
    #define STDIN_FILENO 0
#endif

#ifdef WIN32
    #define NETHACK_SHARED_LIB "nethack.dll"
#else
    #define NETHACK_SHARED_LIB "libnethack.so"
#endif

extern "C" {
#include "hack.h"
#include "nledl.h"
}

class ScopedTC
{
  public:
    ScopedTC() : old_{}
    {
        tcgetattr(STDIN_FILENO, &old_);
        struct termios tty = old_;
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tcsetattr(STDIN_FILENO, TCSANOW, &tty);
    }

    ~ScopedTC()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    }

  private:
    struct termios old_;
};

void
play(nle_ctx_t *nle, nle_obs *obs)
{
    char i;
    while (!obs->done) {
        for (int r = 0; r < ROWNO; ++r) {
            for (int c = 0; c < COLNO - 1; ++c)
                std::cout << obs->chars[r * (COLNO - 1) + c];
            std::cout << std::endl;
        }
        for (int i = 0; i < 23; ++i) {
            std::cout << obs->blstats[i] << " ";
        }
        std::cout << std::endl;
        read(STDIN_FILENO, &obs->action, 1);
        if (obs->action == 'r')
            nle_reset(nle, obs, nullptr, nullptr);
        nle = nle_step(nle, obs);
    }
}

void
randplay(nle_ctx_t *nle, nle_obs *obs)
{
    int actions[] = {
        13, 107, 108, 106, 104, 117, 110, 98, 121,
        75, 76,  74,  72,  85,  78,  66,  89,
    };
    size_t n = sizeof(actions) / sizeof(actions[0]);

    for (int i = 0; !obs->done && i < 10000; ++i) {
        obs->action = actions[rand() % n];
        nle = nle_step(nle, obs);
    }
    if (!obs->done) {
        std::cerr << "Episode didn't end after 10000 steps, aborting."
                  << std::endl;
    }
}

void
randgame(nle_ctx_t *nle, nle_obs *obs, const int no_episodes)
{
    for (int i = 0; i < no_episodes; ++i) {
        randplay(nle, obs);
        if (i < no_episodes - 1)
            nle_reset(nle, obs, nullptr, nullptr);
    }
}

int
main(int argc, char **argv)
{
    nle_obs obs{};
    constexpr int dungeon_size = ROWNO * (COLNO - 1);
    short glyphs[dungeon_size];
    obs.glyphs = &glyphs[0];

    unsigned char chars[dungeon_size];
    obs.chars = &chars[0];

    unsigned char colors[dungeon_size];
    obs.colors = &colors[0];

    unsigned char specials[dungeon_size];
    obs.specials = &specials[0];

    unsigned char message[256];
    obs.message = &message[0];

    long blstats[NLE_BLSTATS_SIZE];
    obs.blstats = &blstats[0];

    int program_state[NLE_PROGRAM_STATE_SIZE];
    obs.program_state = &program_state[0];

    int internal[NLE_INTERNAL_SIZE];
    obs.internal = &internal[0];

    std::unique_ptr<FILE, int (*)(FILE *)> ttyrec(
        fopen("nle.ttyrec.bz2", "a"), fclose);

    ScopedTC tc;
    nle_ctx_t *nle = nle_start(NETHACK_SHARED_LIB, &obs, ttyrec.get(), nullptr);
    if (argc > 1 && argv[1][0] == 'r') {
        randgame(nle, &obs, 3);
    } else {
        play(nle, &obs);
        nle_reset(nle, &obs, nullptr, nullptr);
        play(nle, &obs);
    }
    nle_end(nle);
}

// TODO: There must be a better solution than this, but linking the static
// termiwin.lib is not successful in cmake, nor is including termiwin.c to
// the list of source files for the target
#ifdef WIN32

typedef struct COM {
  HANDLE hComm;
  int fd; //Actually it's completely useless
  char port[128];
} COM;

DCB SerialParams = { 0 }; //Initializing DCB structure
struct COM com;
COMMTIMEOUTS timeouts = { 0 }; //Initializing COMMTIMEOUTS structure

//LOCAL functions

//nbyte 0->7

int getByte(tcflag_t flag, int nbyte, int nibble) {

  int byte;
  if (nibble == 1)
    byte = (flag >> (8 * (nbyte)) & 0x0f);
  else
    byte = (flag >> (8 * (nbyte)) & 0xf0);
  return byte;
}

//INPUT FUNCTIONS

int getIXOptions(tcflag_t flag) {

#define i_IXOFF 0x01
#define i_IXON 0x02
#define i_IXOFF_IXON 0x03
#define i_PARMRK 0x04
#define i_PARMRK_IXOFF 0x05
#define i_PARMRK_IXON 0x06
#define i_PARMRK_IXON_IXOFF 0x07

  int byte = getByte(flag, 1, 1);

  return byte;
}

//LOCALOPT FUNCTIONS

int getEchoOptions(tcflag_t flag) {

#define l_NOECHO 0x00
#define l_ECHO 0x01
#define l_ECHO_ECHOE 0x03
#define l_ECHO_ECHOK 0x05
#define l_ECHO_ECHONL 0x09
#define l_ECHO_ECHOE_ECHOK 0x07
#define l_ECHO_ECHOE_ECHONL 0x0b
#define l_ECHO_ECHOE_ECHOK_ECHONL 0x0f
#define l_ECHO_ECHOK_ECHONL 0x0d
#define l_ECHOE 0x02
#define l_ECHOE_ECHOK 0x06
#define l_ECHOE_ECHONL 0x0a
#define l_ECHOE_ECHOK_ECHONL 0x0e
#define l_ECHOK 0x04
#define l_ECHOK_ECHONL 0x0c
#define l_ECHONL 0x08

  int byte = getByte(flag, 1, 1);
  return byte;
}

int getLocalOptions(tcflag_t flag) {

#define l_ICANON 0x10
#define l_ICANON_ISIG 0x50
#define l_ICANON_IEXTEN 0x30
#define l_ICANON_NOFLSH 0x90
#define l_ICANON_ISIG_IEXTEN 0x70
#define l_ICANON_ISIG_NOFLSH 0xd0
#define l_ICANON_IEXTEN_NOFLSH 0xb0
#define l_ICANON_ISIG_IEXTEN_NOFLSH 0xf0
#define l_ISIG 0x40
#define l_ISIG_IEXTEN 0x60
#define l_ISIG_NOFLSH 0xc0
#define l_ISIG_IEXTEN_NOFLSH 0xe0
#define l_IEXTEN 0x20
#define l_IEXTEN_NOFLSH 0xa0
#define l_NOFLSH 0x80

  int byte = getByte(flag, 1, 0);
  return byte;
}

int getToStop(tcflag_t flag) {

#define l_TOSTOP 0x01

  int byte = getByte(flag, 1, 1);
  return byte;
}

//CONTROLOPT FUNCTIONS

int getCharSet(tcflag_t flag) {

  //FLAG IS MADE UP OF 8 BYTES, A FLAG IS MADE UP OF A NIBBLE -> 4 BITS, WE NEED TO EXTRACT THE SECOND NIBBLE (1st) FROM THE FIFTH BYTE (6th).
  int byte = getByte(flag, 1, 1);

  switch (byte) {

  case 0X0:
    return CS5;
    break;

  case 0X4:
    return CS6;
    break;

  case 0X8:
    return CS7;
    break;

  case 0Xc:
    return CS8;
    break;

  default:
    return CS8;
    break;
  }
}

int getControlOptions(tcflag_t flag) {

#define c_ALL_ENABLED 0xd0
#define c_PAREVEN_CSTOPB 0x50
#define c_PAREVEN_NOCSTOPB 0x40
#define c_PARODD_NOCSTOPB 0xc0
#define c_NOPARENB_CSTOPB 0x10
#define c_ALL_DISABLED 0x00

  int byte = getByte(flag, 1, 0);
  return byte;
}

int tcgetattr(int fd, struct termios* termios_p) {

  if (fd != com.fd) return -1;
  int ret = 0;

  ret = GetCommState(com.hComm, &SerialParams);

  return 0;
}

int tcsetattr(int fd, int optional_actions, const struct termios* termios_p) {

  if (fd != com.fd) return -1;
  int ret = 0;

  //Store flags into local variables
  tcflag_t iflag = termios_p->c_iflag;
  tcflag_t lflag = termios_p->c_lflag;
  tcflag_t cflag = termios_p->c_cflag;
  tcflag_t oflag = termios_p->c_oflag;

  //iflag

  int IX = getIXOptions(iflag);

  if ((IX == i_IXOFF_IXON) || (IX == i_PARMRK_IXON_IXOFF)) {

    SerialParams.fOutX = TRUE;
    SerialParams.fInX = TRUE;
    SerialParams.fTXContinueOnXoff = TRUE;
  }

  //lflag
  int EchoOpt = getEchoOptions(lflag);
  int l_opt = getLocalOptions(lflag);
  int tostop = getToStop(lflag);

  //Missing parameters...

  //cflags

  int CharSet = getCharSet(cflag);
  int c_opt = getControlOptions(cflag);

  switch (CharSet) {

  case CS5:
    SerialParams.ByteSize = 5;
    break;

  case CS6:
    SerialParams.ByteSize = 6;
    break;

  case CS7:
    SerialParams.ByteSize = 7;
    break;

  case CS8:
    SerialParams.ByteSize = 8;
    break;
  }

  switch (c_opt) {

  case c_ALL_ENABLED:
    SerialParams.Parity = ODDPARITY;
    SerialParams.StopBits = TWOSTOPBITS;
    break;

  case c_ALL_DISABLED:
    SerialParams.Parity = NOPARITY;
    SerialParams.StopBits = ONESTOPBIT;
    break;

  case c_PAREVEN_CSTOPB:
    SerialParams.Parity = EVENPARITY;
    SerialParams.StopBits = TWOSTOPBITS;
    break;

  case c_PAREVEN_NOCSTOPB:
    SerialParams.Parity = EVENPARITY;
    SerialParams.StopBits = ONESTOPBIT;
    break;

  case c_PARODD_NOCSTOPB:
    SerialParams.Parity = ODDPARITY;
    SerialParams.StopBits = ONESTOPBIT;
    break;

  case c_NOPARENB_CSTOPB:
    SerialParams.Parity = NOPARITY;
    SerialParams.StopBits = TWOSTOPBITS;
    break;
  }

  //aflags

  //Missing parameters...

  //special characters

  if (termios_p->c_cc[VEOF] != 0) SerialParams.EofChar = (char)termios_p->c_cc[VEOF];
  if (termios_p->c_cc[VINTR] != 0) SerialParams.EvtChar = (char)termios_p->c_cc[VINTR];

  if (termios_p->c_cc[VMIN] == 1) { //Blocking

    timeouts.ReadIntervalTimeout = 0;         // in milliseconds
    timeouts.ReadTotalTimeoutConstant = 0;    // in milliseconds
    timeouts.ReadTotalTimeoutMultiplier = 0;  // in milliseconds
    timeouts.WriteTotalTimeoutConstant = 0;   // in milliseconds
    timeouts.WriteTotalTimeoutMultiplier = 0; // in milliseconds

  } else { //Non blocking

    timeouts.ReadIntervalTimeout = termios_p->c_cc[VTIME] * 100;         // in milliseconds
    timeouts.ReadTotalTimeoutConstant = termios_p->c_cc[VTIME] * 100;    // in milliseconds
    timeouts.ReadTotalTimeoutMultiplier = termios_p->c_cc[VTIME] * 100;  // in milliseconds
    timeouts.WriteTotalTimeoutConstant = termios_p->c_cc[VTIME] * 100;   // in milliseconds
    timeouts.WriteTotalTimeoutMultiplier = termios_p->c_cc[VTIME] * 100; // in milliseconds
  }

  SetCommTimeouts(com.hComm, &timeouts);

  //EOF

  ret = SetCommState(com.hComm, &SerialParams);
  if (ret != 0)
    return 0;
  else
    return -1;
}

#endif