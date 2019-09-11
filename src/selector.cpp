/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Selector.h
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//#define VP_DEBUG
//#define VP_DEBUG_MODE 50
#include <sot/core/debug.hh>
#ifdef VP_DEBUG
class selector__INIT {
public:
  selector__INIT(void) { dynamicgraph::sot::DebugTrace::openFile(); }
};
selector__INIT selector_initiator;
#endif //#ifdef VP_DEBUG

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>
#include <sot-pattern-generator/exception-pg.h>
#include <sot-pattern-generator/selector.h>

namespace dynamicgraph {
namespace sot {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Selector, "Selector");

Selector::Selector(const std::string &name)
    : Entity(name),
      selectorSIN(NULL, "Selector(" + name + ")::input(uint)::selec") {
  sotDEBUGIN(5);

  signalRegistration(selectorSIN);
  initCommands();

  sotDEBUGOUT(5);
}

Selector::~Selector(void) {
  sotDEBUGIN(5);

  resetSignals(0, 0);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS --------------------------------------------------------- */
/* --- SIGNALS --------------------------------------------------------- */
/* --- SIGNALS --------------------------------------------------------- */
#define SOT_CALL_SIG(sotName, sotType)                                         \
  boost::bind(&Signal<sotType, int>::access, &sotName, _2)

template <class T>
unsigned int Selector::createSignal(const std::string &shortname,
                                    const int &sigId__) {
  sotDEBUGIN(15);

  unsigned int sigId = sigId__;
  /*  If sigId is not valid, it is set to the first free signal. */
  if ((sigId__ < 0) || (sigId > nbSignals))
    for (unsigned int i = 0; i < nbSignals; ++i)
      if (0 == inputsSIN[i].size()) {
        sigId = i;
        break;
      }
  if ((sigId__ < 0) || (sigId > nbSignals))
    return -1;

  /* Set up the input signal vector. */
  std::vector<SignalBase<int> *> &entriesSIN = inputsSIN[sigId];
  for (unsigned int i = 0; i < entriesSIN.size(); ++i) {
    if (NULL != entriesSIN[i]) {
      signalDeregistration(entriesSIN[i]->getName());
      delete entriesSIN[i];
    }
  }
  entriesSIN.resize(nbEntries);

  /* sigDep contains the list of the input signal.
     sigOut depends of these. */
  SignalArray<int> sigDep;
  std::ostringstream signame;

  /* Set the entries. */
  for (unsigned int i = 0; i < nbEntries; ++i) {
    signame.str("");
    signame << "Selector(" << Entity::getName() << ")::input("
            << typeid(T).name() << ")::" << shortname << i;
    SignalPtr<T, int> *sigIn = new SignalPtr<T, int>(NULL, signame.str());
    inputsSIN[sigId][i] = sigIn;

    signalRegistration(*sigIn);
    sigDep << (*sigIn);
  }

  /* Set the output. */
  if (NULL != outputsSOUT[sigId]) {
    signalDeregistration(outputsSOUT[sigId]->getName());
    delete outputsSOUT[sigId];
  }
  signame.str("");
  signame << "Selector(" << Entity::getName() << ")::output("
          << typeid(T).name() << ")::" << shortname;

  SignalTimeDependent<T, int> *sigOut = new SignalTimeDependent<T, int>(
      boost::bind(&Selector::computeSelection<T>,
                  SOT_CALL_SIG(selectorSIN, unsigned int),
                  boost::ref(entriesSIN), _1, _2),
      sigDep << selectorSIN, signame.str());
  outputsSOUT[sigId] = sigOut;
  signalRegistration(*sigOut);
  sigOut->setReady(true);

  sotDEBUGOUT(15);
  return sigId;
}

template <class T>
T &Selector::computeSelection(const unsigned int &sigNum,
                              std::vector<SignalBase<int> *> &entriesSIN,
                              T &res, const int &time) {
  sotDEBUGIN(15);

  sotDEBUG(25) << "Type " << typeid(T).name() << std::endl;

  if (sigNum > entriesSIN.size()) {
    SOT_THROW ExceptionPatternGenerator(
        ExceptionPatternGenerator::SELECTOR_RANK,
        "Rank of the selector is not valid. ",
        "(while calling outputsSOUT with selector"
        "=%d).",
        sigNum);
  }

  sotDEBUG(25) << "Sig name " << entriesSIN[sigNum]->getName() << std::endl;
  SignalPtr<T, int> *sigSpec =
      dynamic_cast<SignalPtr<T, int> *>(entriesSIN[sigNum]);
  if (NULL == sigSpec) {
    SOT_THROW ExceptionPatternGenerator(
        ExceptionPatternGenerator::BAD_CAST,
        "Signal types for IN and OUT uncompatible. ",
        "(while calling outputsSOUT of sig<%d>"
        "with output type %s and input of %s).",
        sigNum, (typeid(T).name()), (typeid(*entriesSIN[sigNum]).name()));
  }

  res = sigSpec->access(time);
  sotDEBUGOUT(15);
  return res;
}

void Selector::resetSignals(const unsigned int &nbEntries__,
                            const unsigned int &nbSignals__) {
  for (std::vector<std::vector<SignalBase<int> *> >::iterator iter =
           inputsSIN.begin();
       iter < inputsSIN.end(); ++iter) {
    for (std::vector<SignalBase<int> *>::iterator iterSig = iter->begin();
         iterSig < iter->end(); ++iterSig) {
      SignalBase<int> *sigPtr = *iterSig;
      if (NULL != sigPtr)
        delete sigPtr;
    }
  }
  inputsSIN.resize(nbSignals__);
  nbSignals = nbSignals__;
  nbEntries = nbEntries__;

  for (std::vector<SignalBase<int> *>::iterator iterSig = outputsSOUT.begin();
       iterSig < outputsSOUT.end(); ++iterSig) {
    SignalBase<int> *sigPtr = *iterSig;
    if (NULL != sigPtr)
      delete sigPtr;
  }
  outputsSOUT.resize(nbSignals);
}

/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */

#define SOT_SELECTOR_CREATE_TYPE(sotType, sotTypeName)                         \
  if (dORc && (type == sotTypeName))                                           \
    selec.createSignal<sotType>(name, sigId);                                  \
  else                                                                         \
    oss << "  - " << sotTypeName << std::endl;

static void displayOrCreate(Selector &selec, bool dORc, std::ostream &os,
                            const std::string &name = "",
                            const std::string &type = "",
                            const int &sigId = -1) {
  sotDEBUGIN(15);
  std::ostringstream oss;

  /* ------------------------------------------------------------------------ */
  /* *** Enter here the type list *** --------------------------------------- */
  /* ------------------------------------------------------------------------ */

  SOT_SELECTOR_CREATE_TYPE(Vector, "vector");
  SOT_SELECTOR_CREATE_TYPE(Matrix, "matrix");
  SOT_SELECTOR_CREATE_TYPE(MatrixHomogeneous, "matrixHomo");

  /* ------------------------------------------------------------------------ */
  /* ------------------------------------------------------------------------ */

  if (!dORc)
    os << "Types available:" << std::endl << oss.str();
  sotDEBUGOUT(15);
}

void Selector::initCommands(void) {
  using namespace command;
  addCommand(
      "reset",
      makeCommandVoid2(
          *this, &Selector::resetSignals,
          docCommandVoid2("Re-set the list MxN in and M out, for M outputs, "
                          "and M possible inputs for each M.",
                          "int (N=nb inputs foreach)", "int (M=nb output)")));
  addCommand("create",
             makeCommandVoid3(
                 *this, &Selector::create,
                 docCommandVoid3(
                     "Create a new set of input and the corresponding output.",
                     "string among authorized values (type)", "str(name)",
                     "int (sig id)")));
  addCommand("getTypeList",
             makeCommandVerbose(
                 *this, &Selector::getTypeList,
                 docCommandVerbose("Get the list of all possible types.")));
}

void Selector::create(const std::string &type, const std::string &name,
                      const int &sigId) {
  std::ostringstream dummy;
  displayOrCreate(*this, true, dummy, name, type, sigId);
}
std::string Selector::getTypeList(void) {
  std::ostringstream sout;
  displayOrCreate(*this, false, sout);
  return sout.str();
}
void Selector::getTypeList(std::ostream &os) { os << getTypeList(); }

void Selector::commandLine(const std::string &cmdLine,
                           std::istringstream &cmdArgs, std::ostream &os) {
  if (cmdLine == "help") {
    os << "Selector: " << std::endl
       << "  - typeList: display the available types. " << std::endl
       << "  - reset <nbEntries> <nbSig>: reset the signal lists. " << std::endl
       << "  - create <sigtype> <signame> <sigid>: "
       << "create a new set of signals" << std::endl;
  } else if (cmdLine == "typeList") {
    displayOrCreate(*this, false, os);
  } else if (cmdLine == "create") {
    std::string type, name;
    int sigId;
    cmdArgs >> type >> name >> std::ws;
    if (cmdArgs.good())
      cmdArgs >> sigId;
    else
      sigId = -1;
    displayOrCreate(*this, true, os, name, type, sigId);

  } else if (cmdLine == "reset") {
    cmdArgs >> std::ws;
    if (cmdArgs.good()) {
      unsigned int nbSig, nbEntries;
      cmdArgs >> nbSig >> std::ws;
      if (cmdArgs.good())
        cmdArgs >> nbEntries;
      else {
        os << "Error: usage is: reset <nbEntries> <nbSig>." << std::endl;
        return;
      }
      resetSignals(nbSig, nbEntries);
    } else {
      os << "Error: usage is: reset <nbEntries> <nbSig>." << std::endl;
      return;
    }
  } else {
  }
}

} // namespace sot
} // namespace dynamicgraph
