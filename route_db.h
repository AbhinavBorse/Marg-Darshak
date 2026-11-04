#pragma once
#include <stdint.h>
#include <stddef.h>
#include "audio_map.h"

struct BusRoute {
  uint16_t routeNo;      // bus route number, e.g. 307
  ClipId   source;       // clip enum from audio_map.h
  ClipId   destination;
};

static const BusRoute ROUTE_DB[] = {
  { 307, CLIP_STOP_VAISHALI,  CLIP_STOP_MAJAS   },
  { 522, CLIP_STOP_MILLENIUM, CLIP_STOP_MAJAS   },
  { 496, CLIP_STOP_MARATHON,  CLIP_STOP_AGARKAR },
  { 403, CLIP_STOP_MAHADA,    CLIP_STOP_AGARKAR },
  { 545, CLIP_STOP_AIROLI,    CLIP_STOP_AGARKAR },
};
static const size_t ROUTE_COUNT = sizeof(ROUTE_DB) / sizeof(ROUTE_DB[0]);

// Returns nullptr if the route is not in the database
static inline const BusRoute* findRoute(uint16_t routeNo) {
  for (size_t i = 0; i < ROUTE_COUNT; i++)
    if (ROUTE_DB[i].routeNo == routeNo) return &ROUTE_DB[i];
  return nullptr;
}