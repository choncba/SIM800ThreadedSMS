/* ********************************************************************************/
/*                                                                                */
/*                                                                                */
/*   SIM800ThreadedSMS.h                                                          */
/*                                                                                */
/*   Modified version from ArduinoThreadedGSM By: Neta Yahav <neta540@gmail.com>  */
/*                                                                                */
/*   Created: 2016/09/20 11:14:02 by Neta Yahav                                   */
/*   Updated: 2021/03/10 by chon.cba                                              */
/*                                                                                */
/*   Github: 
/*   Changes: Text mode instead of PDU mode, Storage to TA, no 
/* ********************************************************************************/

#ifndef __SIM800THREADEDSMS_H__
#define __SIM800THREADEDSMS_H__

#include "DTE.h"
#include <Arduino.h>

// Defaults
#define SIM800THREADEDSMS_DEF_DTE_BUF_SIZ 512
#define SIM800THREADEDSMS_DEF_AT_TIMEOUT  5000  //5000 // Faster than original values
#define SIM800THREADEDSMS_DEF_STA_PON     20000  //10000
#define SIM800THREADEDSMS_DEF_STA_POF     1000

// Use custom values or default ones
#ifndef SIM800THREADEDSMS_DTE_BUFFER_SIZE
#define SIM800THREADEDSMS_DTE_BUFFER_SIZE SIM800THREADEDSMS_DEF_DTE_BUF_SIZ
#endif
#ifndef SIM800THREADEDSMS_AT_TIMEOUT
#define SIM800THREADEDSMS_AT_TIMEOUT SIM800THREADEDSMS_DEF_AT_TIMEOUT
#endif
#ifndef SIM800THREADEDSMS_STARTUP_DELAY
#define SIM800THREADEDSMS_STARTUP_DELAY SIM800THREADEDSMS_DEF_STA_PON
#endif
#ifndef SIM800THREADEDSMS_STARTUP_POWER_OFF_DELAY
#define SIM800THREADEDSMS_STARTUP_POWER_OFF_DELAY SIM800THREADEDSMS_DEF_STA_POF
#endif

#define SIM800THREADEDSMS_INTERVAL_COUNT 3

//#define SIM800THREADEDSMS_DEBUG

#ifdef SIM800THREADEDSMS_DEBUG
	#define DEBUG_GSM_PRINT(x)  	  Serial.print(x)      // Defined in main.cpp
	#define DEBUG_GSM_PRINTLN(x)  	Serial.println(x)
#else
#define DEBUG_GSM_PRINT(x)
#define DEBUG_GSM_PRINTLN(x)
#endif

// SMS Init config detail (See https://www.elecrow.com/wiki/images/2/20/SIM800_Series_AT_Command_Manual_V1.09.pdf):
// ATE0 -> NO ECHO
// CMGF=1 -> SMS in text mode
// CPMS="ME_P" -> SMS prefered storage to TA (Not sim card)
// CNMI=0,0,0,0,0 -> NO messages indications at all
// CMGDA="DELL ALL" -> Delete ALL stored messages
// &W -> Save changes
#define SMS_INIT_CONFIG "ATE0;+CMGF=1;+CPMS=\"ME_P\";+CNMI=0,0,0,0,0;+CMGDA=\"DEL ALL\";&W"

class SIM800ThreadedSMS {
  // variables
public:
  struct NetworkTime {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
  };
  enum ReadMessagesListTypeE {
    READ_TYPE_UNREAD = 0,
    READ_TYPE_READ = 1,
    READ_TYPE_UNSENT = 2,
    READ_TYPE_SENT = 3,
    READ_TYPE_ALL = 4
  };
  enum IntervalSourceE { INTERVAL_CLOCK, INTERVAL_INBOX, INTERVAL_SIGNAL };
  struct SignalLevel {
    int Dbm;
    int Value;
  };

  typedef void (*SIM800ThreadedSMSCallbackSignal)(SIM800ThreadedSMS &, SignalLevel &);
  typedef void (*SIM800ThreadedSMSCallbackClock)(SIM800ThreadedSMS &, NetworkTime &);
  //typedef void (*SIM800ThreadedSMSCallbackIncomingSMS)(SIM800ThreadedSMS &, String &);
  typedef void (*SIM800ThreadedSMSCallbackIncomingSMS)(SIM800ThreadedSMS &, String &, String &);
  typedef void (*SIM800ThreadedSMSCallbackBool)(SIM800ThreadedSMS &, bool);
  typedef void (*SIM800ThreadedSMSCallback)(SIM800ThreadedSMS &);
  struct conf {
    SIM800ThreadedSMSCallbackSignal signal;
    SIM800ThreadedSMSCallbackClock clock;
    SIM800ThreadedSMSCallbackIncomingSMS incoming;
    SIM800ThreadedSMSCallback ready;
    SIM800ThreadedSMSCallback outgoing;
    SIM800ThreadedSMSCallbackBool power;
  };

protected:
private:
  enum StatesStartup {
    STARTUP_POWER_OFF,
    STARTUP_POWER_OFF_DELAY,
    STARTUP_POWER_ON,
    STARTUP_DELAY,
    STARTUP_ENTER_AT,
    STARTUP_CHK_CPIN,
    STARTUP_CHK_CREG,
    STARTUP_CHK_CLTS,
    STARTUP_SET_SMS_MODE,
    STARTUP_CHK_CENG
  };

  enum StatesClock { CLOCK_REQ, CLOCK_VERIFY };

  enum StatesSignal { SIGNAL_REQ, SIGNAL_VERIFY };

  enum StatesInbox {
    READ_REQ,
    //READ_CHK_CMGF,
    //READ_CHK_CPMS,
    READ_CHK_CMGL,
    READ_DELAY_CLEAR_BUFF,
    READ_CHK_CMGR,
    READ_CHK_CMGD
  };

  enum StatesOutbox { 
    SEND_REQ, 
    SEND_CHK_CMGF, 
    SEND_CHK_RDY, 
    SEND_CHK_OK 
  };

  unsigned long tick;

  struct {
    int Index;
  } Message;

  // struct {
  //   String InboxMsgContents;
  //   String OutboxMsgContents;
  // } SMS;
  
  // SMS Data
	struct
	{
		String InboxMsgContents;
		String InboxNumber;
		String OutboxMsgContents;
		String OutboxNumber;
	}SMS;

  Stream &stream;
  DTE dte;

  unsigned long tickSync[SIM800THREADEDSMS_INTERVAL_COUNT];
  unsigned long Intervals[SIM800THREADEDSMS_INTERVAL_COUNT];

  // callbacks
  conf configuration = {NULL, NULL, NULL, NULL, NULL, NULL};

  enum ReqTypes {
    REQ_CLOCK = 1,
    REQ_SIG = 2,
    REQ_INBOX = 4,
    REQ_OUTBOX = 8,
    REQ_STARTUP = 16
  };
  int requests;
  int state;
  int job;
  // functions
public:
  SIM800ThreadedSMS(Stream &stream);
  ~SIM800ThreadedSMS();
  void nextJob();
  void setHandlers(conf config);
  void setInterval(IntervalSourceE source, unsigned long interval);
  // Initialization
  void begin();
  // Call this function for executing thread
  void loop();
  // Requests
  void sendSMS(String &Number, String &Message);
  int getBusy();

protected:
private:
  // States
  void Startup();

  // Threads
  void Clock();
  void Signal();
  void clearReq(int req);
  void Inbox();
  void Outbox();
}; // SIM800ThreadedSMS

#endif //__SIM800THREADEDSMS_H__
