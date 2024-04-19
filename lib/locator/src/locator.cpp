#include "Particle.h"
#include "locator.h"

#if Wiring_Cellular
# include "CellularHelper.h"
#endif

static char requestBuf[256];
static char *requestCur;
static int numAdded = 0;

Locator::Locator() : locatorMode(LOCATOR_MODE_MANUAL), periodMs(10000), eventName("deviceLocator"), publicEvent(false),
	stateTime(0), state(CONNECT_WAIT_STATE), callback(NULL), waitAfterConnect(8000), wifiConsiderIp(true) {

}

Locator::~Locator() {

}

Locator &Locator::withLocateOnce() {
	locatorMode = LOCATOR_MODE_ONCE;
	return *this;
}

Locator &Locator::withLocatePeriodic(unsigned long secondsPeriodic) {
	locatorMode = LOCATOR_MODE_PERIODIC;
	if (secondsPeriodic < 5) {
		secondsPeriodic = 5;
	}
	periodMs = secondsPeriodic * 1000;
	return *this;
}

Locator &Locator::withEventName(const char *name) {
	this->eventName = name;
	return *this;
}

Locator &Locator::withPublicEvent() {
	this->publicEvent = true;
	return *this;
}

Locator &Locator::withSubscribe(LocatorSubscriptionCallback callback, bool onlyThisDevice) {
	this->callback = callback;

	if (onlyThisDevice) {
		snprintf(requestBuf, sizeof(requestBuf), "hook-response/%s/%s", eventName.c_str(), System.deviceID().c_str());
	}
	else {
		snprintf(requestBuf, sizeof(requestBuf), "hook-response/%s", eventName.c_str());
	}
	Particle.subscribe(requestBuf, &Locator::subscriptionHandler, this, MY_DEVICES);

	return *this;
}

Locator &Locator::withWiFiConsiderIp(bool value) {
	wifiConsiderIp = value;
	return *this;
}


void Locator::loop() {
	switch(state) {
	case CONNECT_WAIT_STATE:
		if (Particle.connected()) {
			state = CONNECTED_WAIT_STATE;
			stateTime = millis();
		}
		break;

	case CONNECTED_WAIT_STATE:
		if (millis() - stateTime >= waitAfterConnect) {
			// Wait several seconds after connecting before doing the location
			if (locatorMode == LOCATOR_MODE_ONCE) {
				publishLocation();

				state = IDLE_STATE;
			}
			else
			if (locatorMode == LOCATOR_MODE_MANUAL) {
				state = IDLE_STATE;
			}
			else {
				state = CONNECTED_STATE;
				stateTime = millis() - periodMs;
			}
		}
		break;

	case CONNECTED_STATE:
		if (Particle.connected()) {
			if (millis() - stateTime >= periodMs) {
				stateTime = millis();
				publishLocation();
			}
		}
		else {
			// We have disconnected, rec
			state = CONNECT_WAIT_STATE;
		}
		break;


	case IDLE_STATE:
		// Just hang out here forever (entered only on LOCATOR_MODE_ONCE)
		break;
	}

}

const char *Locator::scan() {
#if Wiring_WiFi
	return wifiScan();
#endif
#if Wiring_Cellular
	return cellularScan();
#endif
}


void Locator::publishLocation() {
	Log.trace("publishLocation");
	const char *scanData = scan();
	Log.trace("scanData=%s", scanData);

	if (scanData[0]) {

		if (Particle.connected()) {
			if (publicEvent) {
				Particle.publish(eventName, scanData);
			}
			else {
				Particle.publish(eventName, scanData, PRIVATE);
			}
		}
	}
}

void Locator::subscriptionHandler(const char *event, const char *data) {
	// event: hook-response/deviceLocator/<deviceid>/0

	if (callback) {
		// float lat, float lon, float accuracy
		char *mutableCopy = strdup(data);
		char *part, *end;
		float lat, lon, accuracy;

		part = strtok_r(mutableCopy, ",", &end);
		if (part) {
			lat = atof(part);
			part = strtok_r(NULL, ",", &end);
			if (part) {
				lon = atof(part);
				part = strtok_r(NULL, ",", &end);
				if (part) {
					accuracy = atof(part);

					(*callback)(lat, lon, accuracy);
				}
			}
		}

		free(mutableCopy);
	}
}



#if Wiring_WiFi

static void wifiScanCallback(WiFiAccessPoint* wap, void* data) {
	// The - 3 factor here to leave room for the closing JSON array ] object }} and the trailing null
	size_t spaceLeft = &requestBuf[sizeof(requestBuf) - 3] - requestCur;

	size_t sizeNeeded = snprintf(requestCur, spaceLeft,
			"%s{\"m\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"s\":%d,\"c\":%d}",
			(requestCur[-1] == '[' ? "" : ","),
			wap->bssid[0], wap->bssid[1], wap->bssid[2], wap->bssid[3], wap->bssid[4], wap->bssid[5],
			wap->rssi, wap->channel);
	if (sizeNeeded <= spaceLeft) {
		// There is enough space to store the whole entry, so save it
		requestCur += sizeNeeded;
		numAdded++;
	}
}


const char *Locator::wifiScan() {

	requestCur = requestBuf;
	numAdded = 0;

	requestCur += sprintf(requestCur, "{\"w\":{\"i\":%s,\"a\":", wifiConsiderIp ? "true" : "false");
	*requestCur++ = '[';

	WiFi.scan(wifiScanCallback);

	*requestCur++ = ']';
	*requestCur++ = '}';
	*requestCur++ = '}';
	*requestCur++ = 0;

	if (numAdded == 0) {
		requestBuf[0] = 0;
	}

	return requestBuf;
}

#endif /* Wiring_WiFi */


#if Wiring_Cellular

static void cellularAddTower(const CellularHelperEnvironmentCellData *cellData) {
	// The - 4 factor here to leave room for the closing JSON array ], object }}, and the trailing null
	size_t spaceLeft = &requestBuf[sizeof(requestBuf) - 4] - requestCur;

	size_t sizeNeeded = snprintf(requestCur, spaceLeft,
			"%s{\"i\":%d,\"l\":%u,\"c\":%d,\"n\":%d}",
			(requestCur[-1] == '[' ? "" : ","),
			cellData->ci, cellData->lac, cellData->mcc, cellData->mnc);

	if (sizeNeeded <= spaceLeft && cellData->lac != 0 && cellData->lac != 65535 && cellData->mcc != 65535 && cellData->mnc != 65535) {
		// There is enough space to store the whole entry, so save it
		requestCur += sizeNeeded;
		numAdded++;
	}

}

const char *Locator::cellularScan() {

	// First try to get info on neighboring cells. This doesn't work for me using the U260
	CellularHelperEnvironmentResponseStatic<4> envResp;
	Log.trace("cellularScan()");
	CellularHelper.getEnvironment(5, envResp);
	Log.trace("getEnvironment(5) %d", envResp.resp);

	if (envResp.resp != RESP_OK) {
		// We couldn't get neighboring cells, so try just the receiving cell
		CellularHelper.getEnvironment(3, envResp);
		Log.trace("getEnvironment(3) %d", envResp.resp);
	}
	// envResp.serialDebug();

	if (envResp.resp == RESP_OK) {
		requestCur = requestBuf;
		numAdded = 0;

		// We know these things fit, so just using sprintf instead of snprintf here
		requestCur += sprintf(requestCur, "{\"c\":{\"o\":\"%s\",", CellularHelper.getOperatorName().c_str());
		// Serial.println(CellularHelper.getOperatorName());
		requestCur += sprintf(requestCur, "\"a\":[");
		cellularAddTower(&envResp.service);

		for (size_t ii = 0; ii < envResp.getNumNeighbors(); ii++)
		{
			cellularAddTower(&envResp.neighbors[ii]);
		}

		*requestCur++ = ']';
		*requestCur++ = '}';
		*requestCur++ = '}';
		*requestCur++ = 0;

		if (numAdded == 0) {
			requestBuf[0] = 0;
		}

		Log.trace(requestCur);
	} else {
		Log.trace("trying CellularGlobalIdentity since CGED failed %d", envResp.resp);
		String oper = CellularHelper.getOperatorName();
		CellularGlobalIdentity cgi = {0};
		cgi.size = sizeof(CellularGlobalIdentity);
		cgi.version = CGI_VERSION_LATEST;

		cellular_result_t res = cellular_global_identity(&cgi, NULL);

		if (res == SYSTEM_ERROR_NONE)
		{
			Log.trace("cellular_global_identity res: %d", res);
			requestCur = requestBuf;
			// We know these things fit, so just using sprintf instead of snprintf here
			requestCur += sprintf(requestCur, "{\"c\":{\"o\":\"%s\",", oper.c_str());

			requestCur += sprintf(requestCur, "\"a\":[");

			requestCur += sprintf(requestCur, "{\"i\":%lu,\"l\":%u,\"c\":%u,\"n\":%u}",
														cgi.cell_id, cgi.location_area_code, cgi.mobile_country_code, cgi.mobile_network_code);

			numAdded++;
			*requestCur++ = ']';
			*requestCur++ = '}';
			*requestCur++ = '}';
			*requestCur++ = 0;
		}
		else
		{
			Log.trace("cellular_global_identity failed %d", res);
			requestBuf[0] = 0;
		}
	}

	return requestBuf;
}

// [static]
/*
String Locator::getLocation() {
	if (!Cellular.ready()) {
		strcpy(requestBuf, "{\"valid\":false}");
		return requestBuf;
	}

	CellularHelperLocationResponse locResp = CellularHelper.getLocation(10000);

	if (locResp.valid) {
		return String::format("{\"valid\":true,\"lat\":%f,\"lon\":%f,\"alt\":%d,\"un\":%d}",
				locResp.lat, locResp.lon, locResp.alt, locResp.uncertainty);
	}
	else {
		return "{\"valid\":false}";
	}
}
*/

#endif /* Wiring_Cellular */





