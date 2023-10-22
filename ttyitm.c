#include <linux/kernel.h>           // printk()
#include <linux/errno.h>            // error codes
#include <linux/module.h>           // THIS_MODULE
#include <linux/of.h>
#include <linux/init.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/delay.h>            // udelay
#include <linux/miscdevice.h>       // misc_register
//#include <linux/serial_core.h>
//#include <linux/serial.h>
//#include <linux/sysfs.h>
#include <linux/console.h>
#include <linux/tty.h>


#define DEBUG 1                  // if uncommented, will write some debug messages to /var/log/kern.log

#define DEVICE_NAME         "ttyit"           // The device will appear at /dev/ttyit
#define TTYIT_MAX_DEVS    8
static unsigned int MajorNumber;
static wait_queue_head_t WaitQueue;
static unsigned int DeviceOpen;
static struct i2c_client *_client;


// ring buffer used for receiving data
enum { RX_BUFF_SIZE = 128 };
static volatile unsigned int RxTail = 0;
static volatile unsigned int RxHead = 0;
static unsigned int RxBuff[RX_BUFF_SIZE];

// linear buffer used for transmitting data
enum { TX_BUFF_SIZE = 64 };
static volatile unsigned int TxTail = TX_BUFF_SIZE;
static volatile unsigned int TxHead = TX_BUFF_SIZE;
static unsigned char TxBuff[TX_BUFF_SIZE];

//ako misc device ------------------------------------------------------------------------------
//static int ttyit_open(struct inode* inode, struct file* file);
//static int ttyit_close(struct inode *inode, struct file *file);
//static unsigned int ttyit_poll(struct file* file_ptr, poll_table* wait);
//static ssize_t ttyit_read(struct file* file_ptr, char __user* user_buffer, size_t Count, loff_t* offset);
//static ssize_t ttyit_write(struct file* file_ptr, const char __user* user_buffer, size_t Count, loff_t* offset);
//static long ttyit_ioctl(struct file *fp, unsigned int cmd, unsigned long arg);
//
//// file operations with this kernel module
//static struct file_operations ttyit_fops = {
//    .owner          = THIS_MODULE,
//    .open           = ttyit_open,
//    .release        = ttyit_close,
//    .poll           = ttyit_poll,
//    .read           = ttyit_read,
//    .write          = ttyit_write,
//    .unlocked_ioctl = ttyit_ioctl
//};
//
//static struct miscdevice misc = {
//    .minor = MISC_DYNAMIC_MINOR,
//    .name = DEVICE_NAME,
//    .fops = &ttyit_fops,
//    .mode = S_IRUSR |   // User read
//            S_IWUSR |   // User write
//            S_IRGRP |   // Group read
//            S_IWGRP |   // Group write
//            S_IROTH |   // Other read
//            S_IWOTH     // Other write
//};
// ako tty console ------------------------------------------------------------------
static const struct tty_port_operations ttyit_port_ops;
static struct tty_driver *ttyit_driver;
static struct tty_port ttyit_port;

static int ttyit_open(struct tty_struct *tty, struct file *filp);
static void ttyit_close(struct tty_struct *tty, struct file *filp);
static void ttyit_hangup(struct tty_struct *tty);
static int ttyit_write(struct tty_struct *tty, const unsigned char *buf, int count);
static unsigned int ttyit_write_room(struct tty_struct *tty);
static const struct tty_operations ttyit_ops = {
	.open = ttyit_open,
	.close = ttyit_close,
	.hangup = ttyit_hangup,
	.write = ttyit_write,
	.write_room = ttyit_write_room,
};

static struct tty_driver *ttyit_device(struct console *c, int *index);
static struct console ttyit_console = {
	.name   = "ttyit",
	.device = ttyit_device,
	.index  = -1,
};
//static struct console ttyit_console = {
//	.name		= TTYIT_DEVICENAME,
//	.write		= ttyit_console_write,
//	.device		= uart_console_device,
//	.setup		= ttyit_console_setup,
//	.flags		= CON_PRINTBUFFER,
//	.index		= -1,
//	.data		= &ttyit_uart,
//};

#define TTYIT_CONSOLE_DEVICE	(&ttyit_console)




//-----------------------------------------------------------------------------------
#define MAX_MSG_SIZE 16
static int unpack(const u8 *in, const int size, u8 *out) {
  if(size < 2) { return 0; }
  int msg_size = in[0];
  if(msg_size > MAX_MSG_SIZE || msg_size != size - 2) { return 0; }
  u8 chksum = 0;
  for(int i=0; i<msg_size; i++) {
    chksum += in[i+1];
    out[i] = in[i+1];
  }
  if(chksum == in[msg_size+1]) {
    return msg_size;
  } else {
    return 0;
  }
}

static void pack(const u8 *in, const int size, u8 *out) {
  int msg_size = size>MAX_MSG_SIZE ? MAX_MSG_SIZE : size;
  out[0] = msg_size;
  u8 chksum = 0;
  for(int i=0; i< msg_size; i++) {
    chksum += in[i];
    out[i+1] = in[i];
  }
  out[msg_size+1] = chksum;
}

// ===============================================================================================
//
//                                    ttyit_poll
//
// ===============================================================================================
//
// Parameter:
//      file_ptr            Pointer to the open file
//      wait                Timeout structure
//
// Returns:
//      POLLIN              Data is available
//
// Description:
//      Probe the receiver if some data available. Return after timeout anyway.
//
// ===============================================================================================
static unsigned int ttyit_poll(struct file* file_ptr, poll_table* wait) {
  unsigned int RxNext;
#ifdef DEBUG
  pr_info("Poll request\n");
#endif

  if(_client) {
    //int ret = i2c_smbus_read_byte_data(_client, 0x80);
    //pr_info("poll i2c received %x\n", ret);
    __u8 inb[I2C_SMBUS_BLOCK_MAX];
    int ret = i2c_smbus_read_block_data(_client, 128, (u8*)&inb);
    pr_info("poll i2c received %x\n", ret);
    udelay(500); //FIXME debug
    if(ret >= 0) {
      if(ret==1 && inb[0]==0){
        //TODO ESP POSIELA jednu nulu ak nema co poslat
        pr_info("poll zero received %d\n", ret);
        return 0;
      }
      u8 uinb[MAX_MSG_SIZE];
      int msg_size = unpack(inb, ret, (u8*)&uinb);
      if(msg_size==0){return 0;}
      for(int j=0; j<msg_size; j++){
        //printf("%x ", inb[j]);
        //vlozim hodnotu do buffru
        //spin_lock(&SpinLock);
        RxNext = RxHead + 1;
        if (RxNext >= RX_BUFF_SIZE) {
          RxNext = 0;
        }

        if (RxNext != RxTail) {
          // data was received and is available in the receiver holding register
          // ===================================================================
          RxBuff[RxHead] = uinb[j];
          RxHead = RxNext;
#ifdef DEBUG
          pr_notice("poll: One byte received %x. RxHead=%d, RxTail=%d", uinb[j], RxHead, RxTail);
#endif
        } else {
            // buffer overrun. do nothing. just discard the data.
            // eventually todo: if someone needs to know, we can throw an error here
            // =====================================================================
#ifdef DEBUG
          pr_notice("poll: Buffer overrun. RxHead=%d, RxTail=%d", RxHead, RxTail);
#endif
        }
        //spin_unlock(&SpinLock);

      }
      return POLLIN | POLLRDNORM;//TODO neviem co znamenaju tieto konstanty
    } else {
      pr_info("poll nothing received %d\n", ret);
      return 0;
    }
  } else {
    pr_err("poll none i2c client object\n");
    return 0;
  }
////  poll_wait(file_ptr, &WaitQueue, wait);
//  if (RxTail != RxHead) {
//#ifdef DEBUG
//    pr_info("Poll succeeded. RxHead=%d, RxTail=%d\n", RxHead, RxTail);
//#endif
//    return POLLIN | POLLRDNORM;
//  } else {
//#ifdef DEBUG
//    pr_info("Poll timeout\n");
//#endif
//    return 0;
//  }
}

// ===============================================================================================
//
//                                    ttyit_read
//
// ===============================================================================================
//
// Parameter:
//      file_ptr            Pointer to the open file
//      user_buffer         Buffer in user space where to receive the data
//      Count               Number of bytes to read
//      offset              Pointer to a counter that can hold an offset when reading chunks
//
// Returns:
//      Number of bytes read
//
// Description:
//      Called when a process, which already opened the dev file, attempts to read from it, like
//      "cat /dev/ttyit"
//
// ===============================================================================================
static ssize_t ttyit_read(struct file* file_ptr, char __user* user_buffer, size_t Count, loff_t* offset) {
    unsigned int NumBytes;
    //unsigned int result;
    //unsigned long Flags;
    enum { BUFFER_SIZE = 512 };
    char buffer[BUFFER_SIZE];

#ifdef DEBUG
  pr_info("Read request with offset=%d and count=%u\n", (int)*offset, (unsigned int)Count);
#endif

  //TODO ak mam prazdny buffer sam zavolam poll ktory realne cita zo zariadenia
//  if(RxTail == RxHead) {
//    //realne citanie bude asi vlasty thread
//    //zatial staci vycitavat v poll do buffra a read len odovzdavat buffer
//    ttyit_poll(file_ptr, NULL);
//  }

  // collect all bytes received so far from the receive buffer
  // we must convert from a ring buffer to a linear buffer
  // =========================================================
  NumBytes = 0;
  while (RxTail != RxHead && NumBytes < Count) {
    buffer[NumBytes++] = RxBuff[RxTail++];
    if (RxTail >= RX_BUFF_SIZE) {
      RxTail = 0;
    }
  }

  // copying data to user space requires a special function to be called
  // ===================================================================
  if (copy_to_user(user_buffer, buffer, Count) != 0) {
    return -EFAULT;
  }

#ifdef DEBUG
  pr_info("Read finish with %d bytes read\n", NumBytes);
#endif

  return NumBytes;        // the number of bytes actually received
}
    
    
// ===============================================================================================
//
//                                    ttyit_write
//
// ===============================================================================================
//
// Parameter:
//      file_ptr            Pointer to the open file
//      user_buffer         Buffer in user space where to receive the data
//      Count               Number of bytes to write
//      offset              Pointer to a counter that can hold an offset when writing chunks
//
// Returns:
//      Number of bytes written
//
// Description:
//      Called when a process, which already opened the dev file, attempts to write to it, like
//      "echo "hello" > /dev/ttyit"
//
// ===============================================================================================
//static ssize_t ttyit_write(struct file* file_ptr, const char __user* user_buffer, size_t Count, loff_t* offset) {
////    int result;
////    int Timeout;
//    //unsigned long Flags;
////    unsigned int DataWord;
//    //unsigned int IntMask;
//
//#ifdef DEBUG
//    printk(KERN_NOTICE "ttyit: Write request with offset=%d and count=%u\n", (int)*offset, (unsigned int)Count);
//#endif
//
//  struct i2c_client *client = NULL;
//  client = _client;
////  struct i2c_client *client = i2c_verify_client(&misc);
////  struct i2c_client *client = i2c_new_client_device(struct i2c_adapter *adap, struct i2c_board_info const *info);
//  pr_info("i2c client = %x\n", client);
//  if(client && Count > 0) {
////    int reg = 0;
////    int value = 0x41;
////    Count = i2c_smbus_write_byte_data(_client, reg, value);
////    const __u8 values[16] = {0x41,0x42,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x43,0x0a,0x0d,0};
//    //__s32 ret = i2c_smbus_write_quick(file, value);
//    //__s32 ret = i2c_smbus_write_byte(file, value);
//    //__s32 ret = i2c_smbus_write_block_data(client, 0, 16, values);
//    if(Count > MAX_MSG_SIZE) {
//      //FIXME odoslat po castiach - aspon myslim ze je tam limit 32 idealne neist na hranu
//      pr_warn("writen chunk %d longer than i2c message limit\n", Count);
//    }
//    u8 packet[MAX_MSG_SIZE+2];
//    pack(user_buffer, Count, packet);
//    __s32 ret = i2c_smbus_write_block_data(client, 0, Count+2, (u8*)&packet);
//    if(ret<0) {
//#ifdef DEBUG
//      pr_err("i2c write block failed\n");
//#endif
//      return -EFAULT;
//    }
//  }
//
////    // if transmission is still in progress, wait until done
////    // =====================================================
////    Timeout = 500;
////    while (TxTail < TxHead) {
////        if (--Timeout < 0)
////            return -EBUSY;
////        udelay(500);
////    }
////
////    // copying data from user space requires a special function to be called
////    // =====================================================================
////    if (Count > TX_BUFF_SIZE)
////        Count = TX_BUFF_SIZE;
////    result = copy_from_user(TxBuff, user_buffer, Count);
////    if (result > 0)             // not all requested bytes copied
////        Count = result;         // nuber of bytes copied
////    else if (result != 0)
////        return -EFAULT;
////
////    DataWord = TxBuff[0];
////    TxTail = 1;
////    TxHead = Count;
//
//#ifdef DEBUG
//  pr_info("Write finish with %lu bytes written\n", Count);
//#endif
//
//  return Count;        // the number of bytes actually transmitted
//}
    
    

// ===============================================================================================
//
//                                    ttyit_open
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//
// Description:
//      Called when a process tries to open the device file, like "cat /dev/ttyit"
//
// ===============================================================================================
//static int ttyit_open(struct inode* inode, struct file* file) {
//
//#ifdef DEBUG
//  pr_info("Open at at major %d  minor %d", imajor(inode), iminor(inode));
//#endif
//
//  // do not allow another open if already open
//  // =========================================
//  if (DeviceOpen) {
//    return -EBUSY;
//  }
//  DeviceOpen++;
//
//  // reset the ring buffer and the linear buffer
//  // ===========================================
//  RxTail = RxHead = 0;
//  TxTail = TxHead = TX_BUFF_SIZE;
//
//#ifdef DEBUG
//  pr_info("Open finish\n");
//#endif
//
//  return 0;
//}


// ===============================================================================================
//
//                                    ttyit_close
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//
// Description:
//      Called when a process closes the device file.
//
// ===============================================================================================
//static int ttyit_close(struct inode *inode, struct file *file) {
//  pr_info("Close at at major %d  minor %d\n", imajor(inode), iminor(inode));
//
//  DeviceOpen--;
//
//  pr_info("Close finish\n");
//
//  return 0;
//}


// ===============================================================================================
//
//                                    ttyit_ioctl
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//      OK
//
// Description:
//      I/O control. Currently this does nothing.
//      So only return an OK status
//
// ===============================================================================================
static long ttyit_ioctl(struct file *fp, unsigned int cmd, unsigned long arg) {
	return 0;
}

/* Addresses to scan */
static const unsigned short normal_i2c[] = {
	0x55, I2C_CLIENT_END };

/* Return 0 if detection is successful, -ENODEV otherwise */
static int ttyit_detect(struct i2c_client *client, struct i2c_board_info *info) {
#ifdef DEBUG
  printk(KERN_NOTICE "ttyit: detect");
#endif
	return 0;
}

//static int ttyit_probe_new(struct i2c_client *client) {
//#ifdef DEBUG
//  printk(KERN_NOTICE "ttyit: i2c probe new");
//#endif
//  _client = client;
//  return 0;
//}

static int ttyit_probe(struct i2c_client *client, const struct i2c_device_id *id) {
  //zda sa ze vsetci si odkladaju client a ten potom pouzivaju na komunikaciu
#ifdef DEBUG
  printk(KERN_NOTICE "ttyit: i2c probe");
#endif
  _client = client;
  return 0;
}

static void ttyit_remove(struct i2c_client *client) {
#ifdef DEBUG
  printk(KERN_NOTICE "ttyit: i2c remove");
#endif
}

#define TTYIT_DEV_PM_OPS NULL

static const struct i2c_device_id ttyit_id[] = {
	{ "ttyit", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ttyit_id);

#ifdef CONFIG_OF
static const struct of_device_id ttyit_of_ids[] = {
	{ }
};
MODULE_DEVICE_TABLE(of, ttyit_of_ids);
#endif

//static struct i2c_driver ttyit_driver = {
//	.driver = {
//		.name	= "ttyit",
//		.pm = TTYIT_DEV_PM_OPS,
//		.of_match_table = of_match_ptr(ttyit_of_ids),
//	},
//	.id_table	= ttyit_id,
//	.probe_new	= ttyit_probe_new,
//	.remove		= ttyit_remove,
//  /* if device autodetection is needed: */
//	.class		= I2C_CLASS_DEPRECATED,
//	.detect		= ttyit_detect,
//	.address_list	= normal_i2c,
//};
static struct i2c_driver ttyit_i2c_uart_driver = {                                                                                   
  .driver = {
		.name	= "ttyit",
		.of_match_table = of_match_ptr(ttyit_of_ids),
  },
  //prestalo mi to volat probe tak neviem co dalej
	.probe  	= ttyit_probe,
//	.probe_new	= ttyit_probe_new,
	.remove		= ttyit_remove,
	.id_table	= ttyit_id,
  /* if device autodetection is needed: */
	.class		= I2C_CLASS_DEPRECATED,
	.detect		= ttyit_detect,
	.address_list	= normal_i2c,
};
  

// ===============================================================================================
//
//                                    ttyit_init
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//      Major Number of the driver
//
// Description:
//      Register the device to the kernel by use of the register-chrdev(3) call. Since the first
//      parameter to this call is 0, the system will assign a Major Number by itself. A
//      device name is given and the file_operations structure is also passed to the kernel.
//
// ===============================================================================================
static int __init ttyit_init(void) {
#ifdef DEBUG
  pr_info("init() is called\n");
#endif

	struct tty_driver *driver;
	int ret;

	driver = tty_alloc_driver(1,
		TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW |
		TTY_DRIVER_UNNUMBERED_NODE);
	if (IS_ERR(driver))
		return PTR_ERR(driver);

	tty_port_init(&ttyit_port);
	ttyit_port.ops = &ttyit_port_ops;

	driver->driver_name = "ttyit";
	driver->name = "ttyit";
	driver->type = TTY_DRIVER_TYPE_CONSOLE;
	driver->init_termios = tty_std_termios;
	driver->init_termios.c_oflag = OPOST | OCRNL | ONOCR | ONLRET;
	tty_set_operations(driver, &ttyit_ops);
	tty_port_link_device(&ttyit_port, driver, 0);

	ret = tty_register_driver(driver);
	if (ret < 0) {
		tty_driver_kref_put(driver);
		tty_port_destroy(&ttyit_port);
		return ret;
	}

	ttyit_driver = driver;
	register_console(&ttyit_console);

//	return 0;

  int result;
//
//  // Dynamically allocate a major number for the device
//  // ==================================================
//  MajorNumber = register_chrdev(0, DEVICE_NAME, &ttyit_fops);
//  if (MajorNumber < 0) {
//    printk(KERN_WARNING "ttyit: can\'t register character device with errorcode = %i\n", MajorNumber);
//    return MajorNumber;
//  }
//
//#ifdef DEBUG
//  printk(KERN_NOTICE "ttyit: registered character device with major number = %i and minor numbers 0...255\n", MajorNumber);
//#endif
//
//  // Register the device driver. We are using misc_register instead of
//  // device_create so we are able to set the attributes to rw for everybody
//  // ======================================================================
//  result = misc_register(&misc);
//  if (result) {
//    unregister_chrdev(MajorNumber, DEVICE_NAME);
//    printk(KERN_ALERT "ttyit: Failed to create the device\n");
//    return result;
//  }
//	
//  // set up a queue for waiting
//  // ==========================
////  init_waitqueue_head(&WaitQueue);

#ifdef DEBUG
  pr_info("i2c add driver\n");
#endif
  //vola i2c_register_dirver
  result = i2c_add_driver(&ttyit_i2c_uart_driver);
  if (result < 0) {
    pr_err("failed to init ttyit i2c --> %d\n", result);
    //goto err_i2c;
    //FIXME unregister uart driver
    return result;
  }
  pr_info("i2c add driver result %d\n", result);

//  DeviceOpen = 0;

#ifdef DEBUG
  pr_info("device created correctly\n");
#endif

  return result;
}
    
    
// ===============================================================================================
//
//                                    ttyit_exit
//
// ===============================================================================================
// Parameter:
//
// Returns:
//
// Description:
//      Unmap the I/O, free the IRQ and unregister the device
//
// ===============================================================================================
static void __exit ttyit_exit(void) {
  pr_info("unregister_device()\n");

//  misc_deregister(&misc);
//  unregister_chrdev(MajorNumber, DEVICE_NAME);
//  MajorNumber = 0;

  i2c_del_driver(&ttyit_i2c_uart_driver);

  unregister_console(&ttyit_console);
	tty_unregister_driver(ttyit_driver);
	tty_driver_kref_put(ttyit_driver);
	tty_port_destroy(&ttyit_port);
}

// ===============================================================================================
//
//                                module_init()      module_exit()
//
// ===============================================================================================
module_init(ttyit_init);
module_exit(ttyit_exit);



MODULE_AUTHOR("Peter Kutak");
MODULE_DESCRIPTION("i2c terminal driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static int ttyit_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(&ttyit_port, tty, filp);
}
static void ttyit_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(&ttyit_port, tty, filp);
}
static void ttyit_hangup(struct tty_struct *tty)
{
	tty_port_hangup(&ttyit_port);
}
static int ttyit_write(struct tty_struct *tty, const unsigned char *buf,
			 int count)
{
  struct i2c_client *client = NULL;
  client = _client;
  pr_info("i2c client = %x\n", client);
  if(client && count > 0) {
    if(count > MAX_MSG_SIZE) {
      //FIXME odoslat po castiach - aspon myslim ze je tam limit 32 idealne neist na hranu
      pr_warn("writen chunk %d longer than i2c message limit\n", count);
    }
    u8 packet[MAX_MSG_SIZE+2];
    pack(buf, count, packet);
    s32 ret = i2c_smbus_write_block_data(client, 0, count+2, (u8*)&packet);
    if(ret<0) {
#ifdef DEBUG
      pr_err("i2c write block failed\n");
#endif
      return -EFAULT;
    }
  }

#ifdef DEBUG
  pr_info("Write finish with %lu bytes written\n", count);
#endif

  return count;        // the number of bytes actually transmitted
}
static unsigned int ttyit_write_room(struct tty_struct *tty)
{
  //TODO kolko miesta mam v buffri
	return MAX_MSG_SIZE;
}
static struct tty_driver *ttyit_device(struct console *c, int *index)
{
	*index = 0;
	return ttyit_driver;
}

