/***********************************************************************/
/* M O D U L E N A M E  : PCCAN                                         */
/*                                                                      */
/* D E S C R I P T I O N : functions concerning the CAN 80C200          */
/*                                                                      */
/*                                                                      */
/* N O T E S :         ---                                              */
/*                                                                      */
/*                                                                      */
/* M O D I F I C A T I O N   R E C O R D :                              */
/* Date :     Designer :        Modification :                          */
/* 26-04-93   A.H.A. Blom       Initial program                         */
/* 16-12-93   A.H.A. Blom       Version for MANUS II                    */
/************************************************************************/

/* ---- Includes : ---------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/io.h>
#include <unistd.h>
#include <ncurses.h>

/* ---- Defines : ---------------------------------------------------- */
#define CONT_REG 0x300
#define COMM_REG 0x301
#define STAT_REG 0x302
#define INT_REG  0x303
#define ACC_REG  0x304
#define ACM_REG  0x305
#define BT0_REG  0x306
#define BT1_REG  0x307
#define OUTP_REG 0x308
#define TEST_REG 0x309

#define TXID_REG 0x30A
#define TRTR_REG 0x30B
#define TXB1     0x30C
#define TXB2     0x30D
#define TXB3     0x30E
#define TXB4     0x30F
#define TXB5     0x310
#define TXB6     0x311
#define TXB7     0x312
#define TXB8     0x313

#define RXID_REG 0x314
#define RRTR_REG 0x315
#define RXB1     0x316
#define RXB2     0x317
#define RXB3     0x318
#define RXB4     0x319
#define RXB5     0x31A
#define RXB6     0x31B
#define RXB7     0x31C
#define RXB8     0x31D

#define CLKD_REG 0x31F

#define ERROR_ID  1
#define EXIT      'o'

#define CARTESIAN  1
#define SET_ZERO   2
#define JOINT      4
#define FOLD_OUT   5
#define FOLD_IN    6

#define STUCK_GRIPPER        0
#define NOT_ATTACHE          1
#define ARM_FOLDED_STRECHED  2
#define BLOCKED_DOF          3
#define MAX_M1_ROTATION      4
#define MOVEMENT_ERROR       5
#define MEMORY186_FULL       6

#define MAX_CART             70
#define MAX_JOINT             2
#define MAX_JOINT_GRIP       10
#define MAX_CART_GRIP        12
#define MAX_JOINT_YPR         3
#define MAX_CART_YPR          6


/* ---- Globals : ----------------------------------------------------- */
int pos[8];
int speed[8];
char cbox;
unsigned char manus_status, manus_message;
int error;
unsigned char actual_cbox;

typedef struct
{ unsigned int ident;
  char rtr, len, dat[8];
} message;

void caninit(void);
void transmit(message *package);
void receive(message *package);
void decode(message *rcvmessage, message *xmitmessage);
void help();
void print_status();
void print_fold_status();
void readkey();
void caninit();


int kbhit(void)
{
    int ch = getch();

    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } else {
        return 0;
    }
}



/****************************************************************************/
/*                          Main                                            */
/****************************************************************************/
int main()
{ message rcvmessage, xmitmessage;
  char i;

  printf("%c[2J",27);  /* clear screen */
  cbox = 0;
  printf("\n\nM A N U S  II   P C   C A N    P R O G R A M\n");
  printf(__DATE__);printf("\n\n");

  // get permision for writing to I/O ports
  if(ioperm(0x300, 31, 1)) {
    printf("Access denied, quitting\n");
    return(-1);
  }
  caninit();
  help();

  while(1)
  { while(!kbhit())
    { for (i = 0; i < 3; i++)
      { receive(&rcvmessage);
        decode(&rcvmessage, &xmitmessage);
        transmit(&xmitmessage);
        print_status();
      }
    }
    readkey();
  }
}


/****************************************************************************/
/*                 D E C O D E                                              */
/****************************************************************************/
void decode(message *rcvmessage, message *xmitmessage)
{ int i;
  static char echo = 1;
  switch(rcvmessage->ident)
  { case 0x350:
      manus_status   = rcvmessage->dat[0];
      manus_message  = rcvmessage->dat[1];

      if ((manus_status & 0x80) == 0x80)  /* message 186 */
      { if ((manus_message == 2) && (echo == 1))
        { printf("MANUS gripper is ready\n");
          echo = 2;
        }
        if ((manus_message == 3) && (echo == 2))
        { printf("MANUS absolute angles are ready\n");
          echo = 0;
        }
      }

      pos[1] = ((unsigned int)rcvmessage->dat[2] << 8) +
               (unsigned char)rcvmessage->dat[3];
      pos[2] = ((unsigned int)rcvmessage->dat[4] << 8) +
               (unsigned char)rcvmessage->dat[5];
      pos[3] = ((unsigned int)rcvmessage->dat[6] << 8) +
               (unsigned char)rcvmessage->dat[7];

      xmitmessage->ident = rcvmessage->ident;
      xmitmessage->rtr   = 0;
      xmitmessage->len   = 0;
      break;

    case 0x360:
      pos[4] = ((unsigned int)rcvmessage->dat[0] << 8) +
               (unsigned char)rcvmessage->dat[1];
      pos[5] = ((unsigned int)rcvmessage->dat[2] << 8) +
               (unsigned char)rcvmessage->dat[3];
      pos[6] = ((unsigned int)rcvmessage->dat[4] << 8) +
               (unsigned char)rcvmessage->dat[5];
      pos[7] = ((unsigned int)rcvmessage->dat[6] << 8) +
               (unsigned char)rcvmessage->dat[7];

      xmitmessage->ident = rcvmessage->ident;
      xmitmessage->rtr   = 0;
      xmitmessage->len   = 0;
      break;

    case 0x37F:
      xmitmessage->rtr = 0;    /* for all cboxes */
      switch(cbox)
      { case CARTESIAN:
          xmitmessage->ident = 0x371;
          xmitmessage->len = 8;
          for(i = 0; i < xmitmessage->len; i++)
            xmitmessage->dat[i] = speed[i];
          break;

        case JOINT:
          xmitmessage->ident = 0x374;
          xmitmessage->len = 8;
          for(i = 0; i < xmitmessage->len; i++)
            xmitmessage->dat[i] = speed[i];
          break;

        case FOLD_OUT:
          xmitmessage->ident = 0x375;
          xmitmessage->len = 0;
          break;

        case FOLD_IN:
          xmitmessage->ident = 0x376;
          xmitmessage->len = 0;
          break;

        default:     /* do not move */
          xmitmessage->ident = 0x370;
          xmitmessage->len = 0;
          break;
      } /* end of switch cbox */
      break;
  }  /* end of switch rcvmessage */
}


/****************************************************************************/
/*                     print status                                         */
/****************************************************************************/
void print_status()
{ int i;

  actual_cbox = manus_status & 0x0F;

  switch(actual_cbox)
  { case 0: printf("No move mode\n"); break;
    case 1: printf("Cartesian mode\n"); break;
    case 4: printf("Joint mode\n"); break;
    case 5: printf("Fold out mode\n"); break;
    case 6: printf("Fold in mode\n"); break;
    default: printf("Unknown mode\n"); break;
  }

  if ((manus_status & 0xC0) == 0xC0)  /* error 186 */
  { switch(manus_message)
    { default:
        printf("ERROR: %d\n",manus_message);
        break;
    }
  }
  else if ((manus_status & 0x40) == 0x40)  /* warning 186 */
  { switch(manus_message)
    { case STUCK_GRIPPER:
        printf("WARNING: stuck gripper\n");
        break;
      case NOT_ATTACHE:
        printf("WARNING: not allowed to attache\n");
        break;
      case ARM_FOLDED_STRECHED:
        printf("WARNING: arm folded or streched\n");
        break;
      case BLOCKED_DOF:
        printf("WARNING: blocked dof\n");
        break;
      case MAX_M1_ROTATION:
        printf("WARNING: max rotation M1 reached\n");
        break;
      case MOVEMENT_ERROR:
        printf("WARNING: MANUS moving without input\n");
        break;
      case MEMORY186_FULL:
        printf("WARNING: memory buffer full\n");
        break;
      default:
        printf("WARNING: unknown warning\n");
        break;
    }
  }
  else if ((manus_status  & 0x80) == 0x80)  /* message 186 */
  { if (manus_message == 0) printf("MANUS folded\n");
    if (manus_message == 1) printf("MANUS unfolded\n");
  }

  if (error == ERROR_ID)
    printf("ERROR: wrong CAN identifier received\n");
}

/****************************************************************************/
/*                     print fold status                                    */
/****************************************************************************/
void print_fold_status()
{ int i;
  if ((manus_status  & 0x80) == 0x80)  /* message 186 */
  { if (manus_message == 0) printf("F\n");
    else if (manus_message == 1) printf("U\n");
  }
  else printf("No fold info\n");
}


/****************************************************************************/
/*                 Readkey                                                 */
/****************************************************************************/
void readkey()
{ int ch,i;
  ch = getch();
  switch(ch)
  /********** control boxes ************/
  { case '0':
      cbox = 0; break;
    case '1':
      cbox = 1; break;
    case '4':
      cbox = 4; break;
    case '5':
      cbox = 5; break;
    case '6':
      cbox = 6; break;
   /********* movements *********/
    case 'q':
      if (cbox == 1) speed[1] = MAX_CART;
      else speed[1] = MAX_JOINT;
      break;
    case 'a':
      if (cbox == 1) speed[1] = -MAX_CART;
      else speed[1] = -MAX_JOINT;
      break;
    case 'w':
      if (cbox == 1) speed[2] = MAX_CART;
      else speed[2] = MAX_JOINT;
      break;
    case 's':
      if (cbox == 1) speed[2] = -MAX_CART;
      else speed[2] = -MAX_JOINT;
      break;
    case 'e':
      if (cbox == 1) speed[3] = MAX_CART;
      else speed[3] = MAX_JOINT;
      break;
    case 'd':
      if (cbox == 1) speed[3] = -MAX_CART;
      else speed[3] = -MAX_JOINT;
      break;
    case 'r':
      if (cbox == 1) speed[4] = MAX_CART_YPR;
      else speed[4] = MAX_JOINT_YPR;
      break;
    case 'f':
      if(cbox==1) speed[4] = -MAX_CART_YPR;
      else speed[4] = -MAX_JOINT_YPR;
      break;
    case 't':
      if (cbox == 1) speed[5] = MAX_CART_YPR;
      else speed[5] = MAX_JOINT_YPR;
      break;
    case 'g':
      if (cbox == 1) speed[5] = -MAX_CART_YPR;
      else speed[5] = -MAX_JOINT_YPR;
      break;
    case 'y':
      if (cbox == 1) speed[6] = MAX_CART_YPR;
      else speed[6] = MAX_JOINT_YPR;
      break;
    case 'h':
      if (cbox == 1) speed[6] = -MAX_CART_YPR;
      else speed[6] = -MAX_JOINT_YPR;
      break;
    case 'u':
      if (cbox == 1) speed[7] = MAX_CART_GRIP;
      else speed[7] = MAX_JOINT_GRIP;
      break;
    case 'j':
      if (cbox == 1) speed[7] = -MAX_CART_GRIP;
      else speed[7] = -MAX_JOINT_GRIP;
      break;
    case ' ':
      for(i=0;i<8;i++)
        speed[i] = 0;
      if ((cbox == 6) || (cbox == 5))
        cbox = 0;
      break;

   /*********** general purpose ***********/
    case 'p':
      printf("Controlbox: %d   ", (actual_cbox = manus_status & 0x0F));
      printf("Position: ");
      for(i = 1; i < 8; i++)
        printf("%5d ",pos[i]);
      printf("\n");
      break;
    case 'i':
      printf("M A N U S  S T A T U S:\n\n");
      print_status();
      break;
    case 'l':
      print_fold_status();
      break;
    case EXIT:
      printf("... Done ...\n");
      exit(1);
    case '?':
      help();
      break;
    default:
      for(i=0;i<8;i++)
        speed[i] = 0;
      if ((cbox == 6) || (cbox == 5))
        cbox = 0;
      break;
  }   /* end of switch */
}  /* end of readkey */


/****************************************************************************/
/*                        Help                                              */
/****************************************************************************/
void help()
{  printf("type '?' for help\n");
   printf("type 'o' for quit\n");
   printf("type 'p' for printing of the positions\n");
   printf("type 'i' for manus status info\n");
   printf("type 'l' for fold status\n");
   printf("type '1' for carthesian control\n");
   printf("type '4' for joint control\n");
   printf("type '5' for fold out\n");
   printf("type '6' for fold in\n\n");

   printf("type 'q/a' for A1 or X (depending the mode)\n");
   printf("type 'w/s' for A2 or Y (depending the mode)\n");
   printf("type 'e/d' for A3 or Z (depending the mode)\n");
   printf("type 'r/f' for A4 or yaw (depending the mode)\n");
   printf("type 't/g' for A5 or pitch (depending the mode)\n");
   printf("type 'y/h' for A6 or roll (depending the mode)\n");
   printf("type 'u/j' for gripper open/close (A7)\n\n");
   printf("type any other key for STOP movements\n");
}


/****************************************************************************/
/*      CANINIT                                                             */
/****************************************************************************/
void caninit(void)
{ int i,p;
  unsigned char r;

  outb(0x00, CONT_REG); /* release reset */

  /* check if CAN hardware is available */
  p = TXID_REG;

  for(i = 0; i <= 10; i++)
  { outb(0xAA, p);
    r = inb(p);
    if (r ^ 0xAA)
    { printf("\nHardware error: CAN driver not found\n");
      exit(1);
    }
    p++;
  }

  outb(0x01, CONT_REG);  /* set reset request bit to configure              */
  outb(0xFF, ACC_REG);  /* acceptation register                            */
  outb(0xFF, ACM_REG);  /* all bits accepted                               */
  outb(0x83, BT0_REG);  /* tscl= 0.5us, sjw= 1.5us                         */
  outb(0x23, BT1_REG);  /* tseg_1= 2us, tseg_2 = 1.5us, one sample per bit */
  outb(0xFA, OUTP_REG);  /* normal output mode, push/pull                   */
  outb(0x00, CONT_REG);  /* release reset request bit                       */
}


/****************************************************************************/
/*               T R A N S M I T                                            */
/****************************************************************************/
void transmit(message *package)
{  unsigned char msb, lsb;
   unsigned int l;
   int i;
   i = ((package->ident >> 3) & 0xFF);
   msb = i;
   i = ((package->ident & 0x07) << 5);
   i += ((package->rtr & 0x01) << 4);
   i += (package->len & 0x0F);
   lsb = i;
   for (l = 0; l < 5000; l++)  /* wait until ready */
   { if (((inb(STAT_REG) & 0x04) >> 2) == 1);
     { break;
     }
   }
   if (l == 5000)
   {  printf("ERROR: Transmit timeout\n");
      exit(1);
   }

   outb(msb, TXID_REG);
   outb(lsb, TRTR_REG);

   if (package->rtr != 1)
   { for(i = 0; i < package->len; i++)
     { outb(package->dat[i], TXB1 + i);
     }
   }

   outb(0x01, COMM_REG);                   /* send it          */
}


/****************************************************************************/
/*                     Receive                                              */
/****************************************************************************/
void receive(message *package)
{ unsigned int msb, lsb, i;
  unsigned long l;
  unsigned char ch;

  for (l = 0; l <= 1000000; l++)   /* wait until something received */
  { if ((inb(STAT_REG) & 0x01) != 0) break;
  }

  if (l >= 1000000)
  { printf("ERROR: Receive timeout\n");
    exit(1);
  }

  msb = inb(RXID_REG);
  lsb = inb(RRTR_REG);

  package->ident = ((msb & 0xff) << 3) + ((lsb & 0xE0) >> 5);
  package->rtr = ((lsb & 0x10) >> 4);
  package->len = (lsb & 0x0F);

  if(package->rtr != 1)
  { for(i = 0; i < package->len; i++)
      package->dat[i] = inb(RXB1 + i);
  }

  outb(0x04, COMM_REG);                  /* release buffer */
}