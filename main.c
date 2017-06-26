/* main.c
 *   by Alex Chadwick
 *
 * Copyright (C) 2017, Alex Chadwick
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <bslug.h>
#include <rvl/cache.h>
#include <rvl/ipc.h>
#include <rvl/Pad.h>

BSLUG_MODULE_GAME("????");
BSLUG_MODULE_NAME("USB GCN Adapter Support");
BSLUG_MODULE_VERSION("v1.0");
BSLUG_MODULE_AUTHOR("Chadderz");
BSLUG_MODULE_LICENSE("BSD");

#define IOS_ALIGN __attribute__((aligned(32)))

#define GCN_CONTROLLER_COUNT 4
#define GCN_TRIGGER_THRESHOLD 170
#define WUP_028_RUMBLE 0x11
#define WUP_028_INIT 0x13
#define WUP_028_DESCRIPTOR_SIZE 0x44
#define WUP_028_ID 0x057e0337
#define WUP_028_ENDPOINT_OUT 0x2
#define WUP_028_ENDPOINT_IN 0x81
#define WUP_028_POLL_SIZE 0x25
#define USB_DEVICE_SIZE 0x180
#define DEV_USB_PATH "/dev/usb/hid"
#define DEV_USB_IOCTL_GET_DEVICE_CHANGE 0
#define DEV_USB_IOCTL_INTERRUPT_IN 3
#define DEV_USB_IOCTL_INTERRUPT_OUT 4
#define DEV_USB_IOCTL_GET_VERSION 6
#define DEV_USB_VERSION 0x00040001

static ios_fd_t usb_hid_fd = -1;
static int started = 0;
static PADData_t gcn_data[GCN_CONTROLLER_COUNT];
static uint32_t usb_devices[USB_DEVICE_SIZE] IOS_ALIGN;
static uint32_t gcn_adapter_id = -1;
struct interrupt_msg {
  uint8_t padding[16];
  uint32_t device;
  uint32_t endpoint;
  uint32_t length;
  void *ptr;
};
int error;
int errorMethod;

static uint8_t init_msg_buffer[1] IOS_ALIGN = { WUP_028_INIT };
static struct interrupt_msg init_msg IOS_ALIGN = {
  .device = -1,
  .endpoint = WUP_028_ENDPOINT_OUT,
  .length = sizeof(init_msg_buffer),
  .ptr = init_msg_buffer
};

static uint8_t poll_msg_buffer[WUP_028_POLL_SIZE] IOS_ALIGN;
static struct interrupt_msg poll_msg IOS_ALIGN = {
  .device = -1,
  .endpoint = WUP_028_ENDPOINT_IN,
  .length = sizeof(poll_msg_buffer),
  .ptr = poll_msg_buffer
};

static uint8_t rumble_msg_buffer[5] IOS_ALIGN = { WUP_028_RUMBLE };
static struct interrupt_msg rumble_msg IOS_ALIGN = {
  .device = -1,
  .endpoint = WUP_028_ENDPOINT_OUT,
  .length = sizeof(rumble_msg_buffer),
  .ptr = rumble_msg_buffer
};

static void onDevOpen(ios_fd_t fd, usr_t unused);
static void callbackIgnore(ios_ret_t ret, usr_t unused);
static void onDevGetVersion(ios_ret_t ret, usr_t unused);
static void onDevUsbChange(ios_ret_t ret, usr_t unused);
static void onDevUsbInit(ios_ret_t ret, usr_t unused);
static void onDevUsbPoll(ios_ret_t ret, usr_t unused);

static void onError(void);

static void myPADInit(void) {
  /* FIXME: Until we've killed all PAD methods, better init still. */
  PADInit();
}
static void myPADRead(PADData_t result[GCN_CONTROLLER_COUNT]) {
  if (!started) {
    started = 1;
    for (int i = 0; i < GCN_CONTROLLER_COUNT; i++)
      gcn_data[i].error = PADData_ERROR_2;
    IOS_OpenAsync(DEV_USB_PATH, 2, onDevOpen, NULL);
  }
  for (int i = 0; i < GCN_CONTROLLER_COUNT; i++)
    result[i] = gcn_data[i];
}
static void myPADControlMotor(int pad, int control) {
  rumble_msg_buffer[pad + 1] = control;
  DCFlushRange(rumble_msg_buffer, 0x20);

  rumble_msg.device = gcn_adapter_id;
  IOS_IoctlAsync(
    usb_hid_fd, DEV_USB_IOCTL_INTERRUPT_OUT,
    &rumble_msg, sizeof(rumble_msg),
    NULL, 0,
    callbackIgnore, NULL
  );
}

BSLUG_MUST_REPLACE(PADInit, myPADInit);
BSLUG_MUST_REPLACE(PADRead, myPADRead);
BSLUG_REPLACE(PADControlMotor, myPADControlMotor);

static void onError(void) {
  usb_hid_fd = -1;
  IOS_CloseAsync(usb_hid_fd, callbackIgnore, NULL);
}

static void onDevOpen(ios_fd_t fd, usr_t unused) {
  (void)unused;
  usb_hid_fd = fd;
  if (fd >= 0)
    IOS_IoctlAsync(
      usb_hid_fd, DEV_USB_IOCTL_GET_VERSION,
      NULL, 0,
      NULL, 0,
      onDevGetVersion, NULL
    );
  else {
    error = fd;
    errorMethod = 1;
  }
}

static void callbackIgnore(ios_ret_t ret, usr_t unused) {
  (void)unused;
  (void)ret;
}

static void onDevGetVersion(ios_ret_t ret, usr_t unused) {
  (void)unused;
  if (ret == DEV_USB_VERSION)
    IOS_IoctlAsync(
      usb_hid_fd, DEV_USB_IOCTL_GET_DEVICE_CHANGE,
      NULL, 0,
      usb_devices, sizeof(usb_devices),
      onDevUsbChange, NULL
    );
  else {
    error = ret;
    errorMethod = 2;
    onError();
  }
}

static void onDevUsbChange(ios_ret_t ret, usr_t unused) {
  (void)unused;
  if (ret >= 0) {
    int found = 0;
    for (int i = 0; i < USB_DEVICE_SIZE && usb_devices[i] < sizeof(usb_devices); i += usb_devices[i] / 4) {
      uint32_t device_id = usb_devices[i + 1];
      if (
        usb_devices[i] == WUP_028_DESCRIPTOR_SIZE
        && usb_devices[i + 4] == WUP_028_ID
      ) {
        found = 1;
        if (gcn_adapter_id != device_id) {
          gcn_adapter_id = device_id;
          init_msg.device = gcn_adapter_id;
          IOS_IoctlAsync(
            usb_hid_fd, DEV_USB_IOCTL_INTERRUPT_OUT,
            &init_msg, sizeof(init_msg),
            NULL, 0,
            onDevUsbInit, NULL
          );
        }
      }
    }
    if (!found) gcn_adapter_id = (uint32_t)-1;
    IOS_IoctlAsync(
      usb_hid_fd, DEV_USB_IOCTL_GET_DEVICE_CHANGE,
      NULL, 0,
      usb_devices, sizeof(usb_devices),
      onDevUsbChange, NULL
    );
  } else {
    error = ret;
    errorMethod = 3;
    onError();
  }
}

static void sendPoll(void) {
  DCFlushRange(poll_msg_buffer, -((-sizeof(poll_msg_buffer)) & ~0x1f));
  IOS_IoctlAsync(
    usb_hid_fd, DEV_USB_IOCTL_INTERRUPT_IN,
    &poll_msg, sizeof(poll_msg),
    NULL, 0,
    onDevUsbPoll, NULL
  );
}

static void onDevUsbInit(ios_ret_t ret, usr_t unused) {
  if (ret >= 0) {
    poll_msg.device = gcn_adapter_id;
    sendPoll();
  } else {
    error = ret;
    errorMethod = 4;
  }
}

static void onDevUsbPoll(ios_ret_t ret, usr_t unused) {
  if (ret >= 0) {
    do {
      if (poll_msg_buffer[0] != 0x21) break;
      for (int i = 0; i < GCN_CONTROLLER_COUNT; i++) {
        uint8_t *data = poll_msg_buffer + (i * 9 + 1);
        if ((data[0] >> 4) != 1 && (data[0] >> 4) != 2) {
          gcn_data[i].error = PADData_ERROR_2;
          continue;
        }
        gcn_data[i].error = 0;
        gcn_data[i].buttons =
          ((data[1] >> 0) & 1 ? PADData_BUTTON_A : 0) |
          ((data[1] >> 1) & 1 ? PADData_BUTTON_B : 0) |
          ((data[1] >> 2) & 1 ? PADData_BUTTON_X : 0) |
          ((data[1] >> 3) & 1 ? PADData_BUTTON_Y : 0) |
          ((data[1] >> 4) & 1 ? PADData_BUTTON_DL : 0) |
          ((data[1] >> 5) & 1 ? PADData_BUTTON_DR : 0) |
          ((data[1] >> 6) & 1 ? PADData_BUTTON_DD : 0) |
          ((data[1] >> 7) & 1 ? PADData_BUTTON_DU : 0) |
          ((data[2] >> 0) & 1 ? PADData_BUTTON_S : 0) |
          ((data[2] >> 1) & 1 ? PADData_BUTTON_Z : 0) |
          (data[7] >= GCN_TRIGGER_THRESHOLD ? PADData_BUTTON_L : 0) |
          (data[8] >= GCN_TRIGGER_THRESHOLD ? PADData_BUTTON_R : 0);
        gcn_data[i].aStickX = data[3] - 128;
        gcn_data[i].aStickY = data[4] - 128;
        gcn_data[i].cStickX = data[5] - 128;
        gcn_data[i].cStickY = data[6] - 128;
        gcn_data[i].sliderL = data[7];
        gcn_data[i].sliderR = data[8];
        gcn_data[i]._unknown8 = 0;
        gcn_data[i]._unknown9 = 0;
      }
    } while (0);
    sendPoll();
  } else {
    error = ret;
    errorMethod = 5;
  }
}

