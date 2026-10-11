#pragma once

void  sensorBegin();
float sensorRead();     // one ping + median of the last 3, cm (999 = nothing in range)
float sensorRefresh();  // 3 fresh pings, forgets old readings
