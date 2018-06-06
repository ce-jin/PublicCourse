// Copyright @2018 Pony AI Inc. All rights reserved.

#include "pnc/agents/agents.h"

#include "pnc/agents/sample/sample_agent.h"
#include "pnc/agents/jcvb/jcvb_agent.h"

// Register sample vehicle agent to a factory with its type name "sample_agent"
static simulation::Registrar<::sample::SampleVehicleAgent> registrar2("sample_agent");
static simulation::Registrar<::jcvb::JcvbVehicleAgent> registrar("jcvb_agent");
