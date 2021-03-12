/* ********************************************************************************/
/*                                                                                */
/*                                                                                */
/*   SIM800ThreadedSMS.cpp                                                          */
/*                                                                                */
/*   Modified version from ArduinoThreadedGSM By: Neta Yahav <neta540@gmail.com>  */
/*                                                                                */
/*   Created: 2016/09/20 11:14:02 by Neta Yahav                                   */
/*   Updated: 2021/03/10 by chon.cba                                              */
/*                                                                                */
/*   Github: 
/*   Changes: Text mode instead of PDU mode, Storage to TA, no 
/* ********************************************************************************/

#include "SIM800ThreadedSMS.h"

SIM800ThreadedSMS::SIM800ThreadedSMS(Stream &stream)
    : stream(stream), dte(stream, SIM800THREADEDSMS_DTE_BUFFER_SIZE) {
  for (int i = 0; i < SIM800THREADEDSMS_INTERVAL_COUNT; i++)
    Intervals[i] = 0;

  job = state = requests = 0;
}

SIM800ThreadedSMS::~SIM800ThreadedSMS() {}

void SIM800ThreadedSMS::nextJob() { job = 0; }

void SIM800ThreadedSMS::setHandlers(conf config) { this->configuration = config; }

void SIM800ThreadedSMS::setInterval(IntervalSourceE source, unsigned long interval) {
  Intervals[source] = interval;
  tickSync[source] = millis();
}

void SIM800ThreadedSMS::begin() { requests = (REQ_STARTUP); }

void SIM800ThreadedSMS::loop() {
  if (dte.getIsBusy())
    return;

  // intervals
  for (int i = 0; i < SIM800THREADEDSMS_INTERVAL_COUNT; i++) {
    if (Intervals[i]) {
      if (millis() - tickSync[i] >= Intervals[i]) {
        switch (i) {
        case INTERVAL_CLOCK:
          requests |= REQ_CLOCK;
          break;
        case INTERVAL_INBOX:
          requests |= REQ_INBOX;
          break;
        case INTERVAL_SIGNAL:
          requests |= REQ_SIG;
          break;
        }
        tickSync[i] = millis();
      }
    }
  }

  if (job == 0) {
    // no assigned job, assign it
    if (requests & REQ_CLOCK)
      job = REQ_CLOCK;
    else if (requests & REQ_SIG)
      job = REQ_SIG;
    else if (requests & REQ_INBOX)
      job = REQ_INBOX;
    else if (requests & REQ_OUTBOX)
      job = REQ_OUTBOX;
    else if (requests & REQ_STARTUP)
      job = REQ_STARTUP;

    if (job) {
      state = 0;
      DEBUG_GSM_PRINT("Job ID: ");
      DEBUG_GSM_PRINTLN(job);
    }
  }

  // execute current job
  if (job == REQ_STARTUP)
    Startup();
  else if (job == REQ_CLOCK)
    Clock();
  else if (job == REQ_SIG)
    Signal();
  else if (job == REQ_INBOX)
    Inbox();
  else if (job == REQ_OUTBOX)
    Outbox();
}

void SIM800ThreadedSMS::sendSMS(String& Number, String& Message){
  requests |= (REQ_OUTBOX);
  SMS.OutboxMsgContents = Message;
  SMS.OutboxNumber = Number;
}

// If return any different from 0, there is a job pending
int SIM800ThreadedSMS::getBusy(){
  return job;
}

void SIM800ThreadedSMS::Startup() {
  int lastState = state;
  switch (state) {
  case STARTUP_POWER_OFF:
    if (this->configuration.power != NULL)
      this->configuration.power(*this, false);
    tick = millis();
    state = STARTUP_POWER_OFF_DELAY;
    break;
  case STARTUP_POWER_OFF_DELAY:
    if (millis() - tick >= SIM800THREADEDSMS_STARTUP_POWER_OFF_DELAY)
      state = STARTUP_POWER_ON;
    break;
  case STARTUP_POWER_ON:
    if (this->configuration.power != NULL)
      this->configuration.power(*this, true);
    // begin delay
    tick = millis();
    state = STARTUP_DELAY;
    break;
  case STARTUP_DELAY:
    if (millis() - tick >= SIM800THREADEDSMS_STARTUP_DELAY) {
      dte.SendCommand("AT\r", SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
      state = STARTUP_ENTER_AT;
    }
    break;
  case STARTUP_ENTER_AT:
    if (dte.getResult() == DTE::EXPECT_RESULT) {
      dte.SendCommand("AT+CPIN?\r", 10000, "OK\r");
      state = STARTUP_CHK_CPIN;
    } else {
      state = STARTUP_POWER_OFF;
    }
    break;
  case STARTUP_CHK_CPIN:
    if (dte.getResult() == DTE::EXPECT_RESULT) {
      if (dte.getBuffer().indexOf("+CPIN: READY") != -1) {
        dte.SendCommand("AT+CREG?\r", SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
        state = STARTUP_CHK_CREG;
      } else {
        state = STARTUP_POWER_OFF;
      }
    } else
      state = STARTUP_POWER_OFF;
    break;
  case STARTUP_CHK_CREG:
    if (dte.getResult() == DTE::EXPECT_RESULT) {
      if ((dte.getBuffer().indexOf(",1") >= 0) ||
          (dte.getBuffer().indexOf(",5") >= 0)) {
        dte.SendCommand("AT+CLTS=1\r", SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
        state = STARTUP_CHK_CLTS;
      } else
        state = STARTUP_POWER_OFF;
    } else
      state = STARTUP_POWER_OFF;
    break;
  case STARTUP_CHK_CLTS:
    if (dte.getResult() == DTE::EXPECT_RESULT) {
      dte.SendCommand("ATE0;+CMGF=1;+CPMS=\"ME_P\";+CNMI=0,0,0,0,0;+CMGDA=\"DEL ALL\";&W\r", SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
      state = STARTUP_SET_SMS_MODE;
    } else
      state = STARTUP_POWER_OFF;
    break;
  case STARTUP_SET_SMS_MODE:
    if (dte.getResult() == DTE::EXPECT_RESULT) {
      dte.SendCommand("AT+CENG=3\r", SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
      state = STARTUP_CHK_CENG;
    } else
      state = STARTUP_POWER_OFF;
    break;  
  case STARTUP_CHK_CENG:
    if (dte.getResult() == DTE::EXPECT_RESULT) {
      requests |= ((REQ_CLOCK) | (REQ_SIG));
      clearReq(REQ_STARTUP);
      for (int i = 0; i < SIM800THREADEDSMS_INTERVAL_COUNT; i++)
        tickSync[i] = millis();
      if (this->configuration.ready != NULL)
        this->configuration.ready(*this);
    } else
      state = STARTUP_POWER_OFF;
    break;
  }
  if (state != lastState) {
    DEBUG_GSM_PRINT(F("STARTUP_STATE: "));
    DEBUG_GSM_PRINTLN(state);
  }
}

void SIM800ThreadedSMS::Clock() {
  String clockTime;
  int lastState = state;
  switch (state) {
  case CLOCK_REQ:
    dte.SendCommand("AT+CCLK?\r", SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
    state = CLOCK_VERIFY;
    break;
  case CLOCK_VERIFY:
    int index = dte.getBuffer().indexOf("+CCLK: ");
    if (index >= 0) {
      // parse clock
      index += 8;
      int endindex;
      endindex = dte.getBuffer().indexOf("+", index);
      if (endindex >= 0)
        clockTime = dte.getBuffer().substring(index, endindex);
      else {
        endindex = dte.getBuffer().indexOf("-", index);
        if (endindex >= 0)
          clockTime = dte.getBuffer().substring(index, endindex);
      }

      if (endindex >= 0) {
        NetworkTime ClockTime;
        ClockTime.year = 2000 + clockTime.substring(0, 2).toInt();
        ClockTime.month = clockTime.substring(3, 5).toInt();
        ClockTime.day = clockTime.substring(6, 8).toInt();
        ClockTime.hour = clockTime.substring(9, 11).toInt();
        ClockTime.minute = clockTime.substring(12, 14).toInt();
        ClockTime.second = clockTime.substring(15, 17).toInt();
        if (this->configuration.clock != NULL)
          this->configuration.clock(*this, ClockTime);
      }
    }
    clearReq(REQ_CLOCK);
    break;
  }
  if (state != lastState) {
    DEBUG_GSM_PRINT(F("CLOCK_STATE: "));
    DEBUG_GSM_PRINTLN(state);
  }
}

void SIM800ThreadedSMS::Signal() {
  int lastState = state;
  switch (state) {
  case SIGNAL_REQ:
    dte.SendCommand("AT+CSQ\r", SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
    state = SIGNAL_VERIFY;
    break;
  case SIGNAL_VERIFY:
    int index = dte.getBuffer().indexOf("+CSQ: ");
    if (index >= 0) {
      // parse signal
      index += 6;
      SignalLevel GsmSignal;
      GsmSignal.Value = dte.getBuffer().substring(index, index + 2).toInt();
      GsmSignal.Dbm = dte.getBuffer().substring(index + 3, index + 5).toInt();
      if (GsmSignal.Value != 0) {
        if (this->configuration.signal != NULL)
          this->configuration.signal(*this, GsmSignal);
      }
    }
    clearReq(REQ_SIG);
    break;
  }
  if (state != lastState) {
    DEBUG_GSM_PRINT(F("SIGNAL_STATE: "));
    DEBUG_GSM_PRINTLN(state);
  }
}

void SIM800ThreadedSMS::clearReq(int req) {
  requests &= ~(req);
  nextJob();
}

void SIM800ThreadedSMS::Inbox() {
  String CMD;
		int lastState = state;
		switch(state)
		{
			case READ_REQ:
				SMS.InboxMsgContents = "";
				dte.SendCommand("AT+CMGL=\"ALL\"\r", SIM800THREADEDSMS_AT_TIMEOUT, ",");
				state = READ_CHK_CMGL;
				break;
			case READ_CHK_CMGL:
				if(dte.getResult() == DTE::EXPECT_RESULT)
				{
					//fetch index
					int indexStart = dte.getBuffer().indexOf("+CMGL: ");
					if (indexStart >= 0)
					{
						Message.Index = dte.getBuffer().substring(indexStart + 7, dte.getBuffer().indexOf(",")).toInt();
						if(Message.Index != 0)
						{
							dte.Delay(2000);
							state = READ_DELAY_CLEAR_BUFF;
						}
					}
				}
				if(state != READ_DELAY_CLEAR_BUFF)
					clearReq(REQ_INBOX);

				break;
			case READ_DELAY_CLEAR_BUFF:
				CMD = "AT+CMGR=";
				CMD += Message.Index;
				CMD += "\r";
				dte.SendCommand(CMD.c_str(), SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
				state = READ_CHK_CMGR;
				break;
			case READ_CHK_CMGR:
				if(dte.getResult() == DTE::EXPECT_RESULT)
				{
					int indexStart = dte.getBuffer().indexOf("+CMGR: ");
					if(indexStart >= 0)
					{
						int indexStartNumber = dte.getBuffer().indexOf("\",\"", indexStart);
						if(indexStartNumber >= 0){
							indexStartNumber+=3;
							int indexEndNumber = dte.getBuffer().indexOf("\",\"", indexStartNumber);
							if(indexEndNumber >= 0){
								SMS.InboxNumber = dte.getBuffer().substring(indexStartNumber, indexEndNumber);
							}
						}
												
						int indexStartTEXT = dte.getBuffer().indexOf("\r\n", indexStart);
						if (indexStartTEXT >= 0)
						{
							indexStartTEXT+=2;
							int indexEndTEXT = dte.getBuffer().indexOf("\r", indexStartTEXT);
							if(indexEndTEXT >= 0)
								SMS.InboxMsgContents = dte.getBuffer().substring(indexStartTEXT, indexEndTEXT);
						}
					}
					CMD = "AT+CMGD=";
					CMD += Message.Index;
					CMD += "\r";
					dte.SendCommand(CMD.c_str(), SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
					state = READ_CHK_CMGD;
				}else
					clearReq(REQ_INBOX);
				break;
			case READ_CHK_CMGD:
				if( (dte.getResult() == DTE::EXPECT_RESULT) && (SMS.InboxMsgContents != ""))
				{
					if(this->configuration.incoming != NULL)
						this->configuration.incoming(*this, SMS.InboxNumber, SMS.InboxMsgContents);
				}
				clearReq(REQ_INBOX);
				break;
		}
		if(state != lastState)
		{
				DEBUG_GSM_PRINT(F("INBOX_STATE: "));
				DEBUG_GSM_PRINTLN(state);
		}
}

void SIM800ThreadedSMS::Outbox() {
String CMD;
		int lastState = state;
		switch(state)
		{
			case SEND_REQ:
				dte.SendCommand("AT+CMGF=1\r", SIM800THREADEDSMS_AT_TIMEOUT, "OK\r");
				state = SEND_CHK_CMGF;
				break;
			case SEND_CHK_CMGF:
				if(dte.getResult() == DTE::EXPECT_RESULT)
				{
					CMD = "AT+CMGS=\"" + SMS.OutboxNumber + "\"\r";
					dte.SendCommand(CMD.c_str(), 15000, "> ");
					state = SEND_CHK_RDY;
				}
				else clearReq(REQ_OUTBOX);
				break;
			case SEND_CHK_RDY:
				if(dte.getResult() == DTE::EXPECT_RESULT)
				{
					CMD = SMS.OutboxMsgContents;
					CMD += (char)26;
					dte.SendCommand(CMD.c_str(), 10000, "OK\r");
					state = SEND_CHK_OK;
				}else clearReq(REQ_OUTBOX);
				break;
			case SEND_CHK_OK:
				if(dte.getResult() == DTE::EXPECT_RESULT)
				{
					if(this->configuration.outgoing != NULL)
						this->configuration.outgoing(*this);
				}
				clearReq(REQ_OUTBOX);
				break;
		}
		if(state != lastState)
		{
				DEBUG_GSM_PRINT(F("OUTBOX_STATE: "));
				DEBUG_GSM_PRINTLN(state);
		}
}
