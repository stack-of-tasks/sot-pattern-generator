from dynamic_graph import plug

from .selector import Selector as Base

print("With meta selector")

# Define a new constructor for the selector entity. The basic Selector(name)
# constructor is still valid.
# Typical use below:
#  % selecSupportFoot = Selector('selecSupportFoot'
#  %                             ,['matrixHomo','pg_H_sf',pg.rightfootref,pg.leftfootref]
#  %                             ,['matrixHomo','wa_H_sf',geom.rf,geom.lf]
#  %                             )


class Selector:
    def __init__(self, name, *signalPlug):
        self = Base(name)
        if len(signalPlug):
            nbOut = len(signalPlug)
            nbIn = len(signalPlug[0]) - 2
            self.reset(nbIn, nbOut)
            idxOut = 0
            for sigOut in signalPlug:
                typeSig = sigOut[0]
                nameSig = sigOut[1]
                self.create(typeSig, nameSig, idxOut)
                idxIn = 0
                for sigIn in sigOut[2:]:
                    plug(sigIn, self.signal(nameSig + str(idxIn)))
                    idxIn += 1
                idxOut += 1
