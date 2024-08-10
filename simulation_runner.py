from single_simulation import single_sim
from behaviours_maker import create_behaviours
import traci
import matplotlib.pyplot as plt
import numpy as np
from mapdata import MapData

CREATE_BEHAVIOURS = False
RUN_SIMULATION = True
NUM_ALGS = 4
IMG_FOLDER = 'UnisaScenario_12h_noincident/'
NUMBER_OF_CARS = 100
NUM_AGENTS = 12
USE_NUM_AGENTS = True
PERC_AGENTS = 0.3
TIME_HORIZON = 5 # keep around 5
SAVE_IMG = False
SAVE_FILE = False

if CREATE_BEHAVIOURS:
    create_behaviours(NUM_ALGS)

if RUN_SIMULATION:
    # Arguments:
    # maximum number of vehicles, percentage of uni-related cars, show GUI, time horizon, simulation step size, include bus, include random cars,
    # starting hour, percentage of agent cars among the uni-related ones, use desired number of agents (instead of percentage), number of agents
    # SIMULATION ENDS WHEN ALL AGENTS HAVE ARRIVED
    speedsplot = []
    fuels = []
    waitingtimes = []
    co2em = []
    noise = []
    numberofsim = 1
    mapdata = MapData()
    if numberofsim<2:
        if not USE_NUM_AGENTS:
            NUM_AGENTS = PERC_AGENTS*NUMBER_OF_CARS
        retval,agents,arrived = single_sim(NUMBER_OF_CARS,1.0,True,TIME_HORIZON,0.1,True,True,12,PERC_AGENTS,mapdata,USE_NUM_AGENTS,NUM_AGENTS,NUM_ALGS=NUM_ALGS,ONLINE=True,CONSIDER_WORKS=False)
        vehicle_speeds = []
        vehicle_fuelconsumptions = []
        vehicle_waitingtimes = []
        vehicle_co2emissions = []
        vehicle_noiseemissions = []
        for r in retval:
            print(r)
            if r[3]>0:
                vehicle_speeds.append(r[2])
                vehicle_fuelconsumptions.append(r[4])
                vehicle_waitingtimes.append(r[5])
                vehicle_co2emissions.append(r[7])
                vehicle_noiseemissions.append(r[6])
        speedsplot.append(sum(vehicle_speeds)/len(vehicle_speeds))
        fuels.append(sum(vehicle_fuelconsumptions)/len(vehicle_fuelconsumptions))
        noise.append(sum(vehicle_noiseemissions)/len(vehicle_noiseemissions))
        waitingtimes.append(sum(vehicle_waitingtimes)/len(vehicle_waitingtimes))
        co2em.append(sum(vehicle_co2emissions)/len(vehicle_co2emissions))
        print('SPAWNED AGENTS || ARRIVED AGENTS')
        print(str(agents)+' || '+str(len(arrived)))
    else: 
        for i in range(numberofsim):
            ssize = 0.1
            if i == 0:
                ssize = 1.0
            retval,agents,arrived = single_sim(NUMBER_OF_CARS,1.0,True,TIME_HORIZON,ssize,True,True,12,0.1*(i),mapdata,False,i,NUM_ALGS=NUM_ALGS,ONLINE=True)
            vehicle_speeds = []
            vehicle_fuelconsumptions = []
            vehicle_waitingtimes = []
            vehicle_co2emissions = []
            vehicle_noiseemissions = []
            for r in retval:
                print(r)
                if r[3]>0:
                    vehicle_speeds.append(r[2])
                    vehicle_fuelconsumptions.append(r[4])
                    vehicle_waitingtimes.append(r[5])
                    vehicle_co2emissions.append(r[7])
                    vehicle_noiseemissions.append(r[6])
            speedsplot.append(sum(vehicle_speeds)/len(vehicle_speeds))
            fuels.append(sum(vehicle_fuelconsumptions)/len(vehicle_fuelconsumptions))
            noise.append(sum(vehicle_noiseemissions)/len(vehicle_noiseemissions))
            waitingtimes.append(sum(vehicle_waitingtimes)/len(vehicle_waitingtimes))
            co2em.append(sum(vehicle_co2emissions)/len(vehicle_co2emissions))
            print('SPAWNED AGENTS || ARRIVED AGENTS')
            print(str(agents)+' || '+str(len(arrived)))
    if SAVE_FILE:
        np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_speeds.ny',speedsplot)
        np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_fuelconsumption.ny',fuels)
        np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_noise.ny',noise)
        np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_co2.ny',co2em)
        np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_waiting.ny',waitingtimes)
    
    if SAVE_IMG:
        plt.plot(speedsplot)
        plt.title('Average speed and controlled cars')
        plt.xlabel('Number of controlled cars')
        plt.ylabel('Average speed')
        plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_speeds.png')
        plt.show()
        plt.plot(fuels)
        plt.title('Fuel consumption and controlled cars')
        plt.xlabel('Number of controlled cars')
        plt.ylabel('Average fuel consumption')
        plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_fuelconsumption.png')
        plt.show()
        plt.plot(waitingtimes)
        plt.title('Waiting time and controlled cars')
        plt.xlabel('Number of controlled cars')
        plt.ylabel('Average waiting time')
        plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_waitingtime.png')
        plt.show()
        plt.plot(co2em)
        plt.title('CO2 emissions and controlled cars')
        plt.xlabel('Number of controlled cars')
        plt.ylabel('Average CO2 emission')
        plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_co2.png')
        plt.show()
        plt.plot(noise)
        plt.title('Noise emission and controlled cars')
        plt.xlabel('Number of controlled cars')
        plt.ylabel('Average noise emissions')
        plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_noise.png')
        plt.show()
    
    