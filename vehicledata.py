
class VehicleData:
    
    def __init__(self,id,alg,depart,dest=''):
        self.id = id
        self.alg = alg
        self.speeds = []
        self.selected_id = 0
        self.dist = 0
        self.dest = dest
        self.depart = int(depart)
        self.fuelconsumption = 0
        self.waitingtime = 0
        self.noiseemission = 0
        self.co2emission = 0
        self.traveltime = 0
        self.influence = 0.5
        self.currentroad = ''
        self.weightened = {}
        self.arrived = False