
class VehicleData:
    
    def __init__(self,id,alg,depart,dest='',route=None):
        self.id = id
        self.alg = alg
        self.speeds = []
        self.selected_id = 0
        self.dist = 0
        self.dest = dest
        self.depart = int(depart)
        self.fuelconsumption = []
        self.waitingtime = [0]
        self.noiseemission = []
        self.co2emission = []
        self.traveltime = 0
        self.influence = 0.5
        self.currentroad = ''
        self.weightened = {}
        self.arrived = False
        self.route = route