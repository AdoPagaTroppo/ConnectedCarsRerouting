# Script for running simulations according to the desired parameters, including vehicle-in-the-loop simulations

HIL = True
if HIL:
    from hil_runner import single_sim
else:
    from single_simulation import single_sim
from behaviours_maker import create_behaviours
import traci
import matplotlib.pyplot as plt
import numpy as np
from mapdata import MapData
from plots import elaborate_and_make_plots
from parameter_gui import gui_create_and_run
from behaviours_maker import create_paths

CREATE_BEHAVIOURS = False # put to True if offline paths and behaviours need to be created
RUN_SIMULATION = True # put to True if simulations must be run
NUM_ALGS = 4 # number of behaviours to take into account
IMG_FOLDER = 'SalernoScenario_12h_incident/' # folder in which files will be saved
SCENARIO = 'Unisa' # name of the scenario
NUMBER_OF_CARS = 20 # number of cars that will enter the simulation
NUM_AGENTS = 1 # number of agents to enter the simulation (this will be ignored if USE_NUM_AGENTS=False)
USE_NUM_AGENTS = True # consider NUM_AGENTS as the number of agents to enter the simulation or use the percentage value expressed by PERC_AGENTS
PERC_AGENTS = 0.3 # percentage of agent vehicles among the vehicles traversing paths of interest
TIME_HORIZON = 3 # time horizon for the crowdsourcing algorithm, keep around 5
SAVE_FILE = False # put to True if files for algorithm evaluation need to be saved

USE_PARAM_GUI = True # put to True if you want to use the parameter GUI

if CREATE_BEHAVIOURS:
    create_behaviours(NUM_ALGS,MapData(SCENARIO),False)
    # create_paths(NUM_ALGS,MapData(SCENARIO),False) # decomment this and comment the previous line if only paths must be created

if RUN_SIMULATION:
    # gather parameters from the GUI if used
    params_s = []
    if USE_PARAM_GUI:
        gui_create_and_run()
        f = open('sim_config.txt','r')
        params = f.readline()
        f.close()
        params_s = params.split(';')
        NUMBER_OF_CARS = int(params_s[0])
    # initialize data structures for saving files
    speedsplot = []
    fuels = []
    waitingtimes = []
    co2em = []
    noise = []
    traveltimes = []
    speedsplot_std = []
    fuels_std = []
    waitingtimes_std = []
    co2em_std = []
    noise_std = []
    traveltimes_std = []
    x_axis = []
    maxrange = 21 if NUMBER_OF_CARS>=20 else 11 # this works under the assumption that simulations will include no less than 10 cars
    numberofsim = range(1,maxrange) if not HIL else [1]
    # numberofsim = [1] # decomment this and comment the previous line if only one simulation must be run
    mapdata = MapData(SCENARIO) # load mapdata
    if len(numberofsim)<2: # run a single simulation
        i = 0
        j = 0
        if not USE_NUM_AGENTS:
            NUM_AGENTS = PERC_AGENTS*NUMBER_OF_CARS
        retval,agents,arrived,sta,ssa,fa,atd,ftd = single_sim(NUMBER_OF_CARS,1.0,True,TIME_HORIZON,0.05,True,True,12,PERC_AGENTS,mapdata,USE_NUM_AGENTS,NUM_AGENTS,NUM_ALGS=NUM_ALGS,ONLINE=False,CONSIDER_WORKS=False)
        # prepare data structures for file saving
        vehicle_speeds = []
        vehicle_fuelconsumptions = []
        vehicle_waitingtimes = []
        vehicle_co2emissions = []
        vehicle_noiseemissions = []
        vehicle_traveltimes = []
        agent_speeds = []
        agent_fuelconsumptions = []
        agent_waitingtimes = []
        agent_co2emissions = []
        agent_noiseemissions = []
        agent_traveltimes = []
        foes_speeds = []
        foes_fuelconsumptions = []
        foes_waitingtimes = []
        foes_co2emissions = []
        foes_noiseemissions = []
        foes_traveltimes = []
        # retrieve data from the simulation
        for r in retval:
            print(r)
            if r[3]>0:
                # add values to data structures for global evaluation
                vehicle_speeds.append(r[2])
                vehicle_fuelconsumptions.append(r[4])
                vehicle_waitingtimes.append(r[5])
                vehicle_co2emissions.append(r[7])
                vehicle_noiseemissions.append(r[6])
                vehicle_traveltimes.append(r[8])
                if r[0].__contains__('agent'): # add values to data structures for agent evaluation
                    agent_speeds.append(r[2])
                    agent_fuelconsumptions.append(r[4])
                    agent_waitingtimes.append(r[5])
                    agent_co2emissions.append(r[7])
                    agent_noiseemissions.append(r[6])
                    agent_traveltimes.append(r[8])
                else: # add values to data structures for non-agent evaluation
                    foes_speeds.append(r[2])
                    foes_fuelconsumptions.append(r[4])
                    foes_waitingtimes.append(r[5])
                    foes_co2emissions.append(r[7])
                    foes_noiseemissions.append(r[6])
                    foes_traveltimes.append(r[8])
        if SAVE_FILE: # save files
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_speeds.ny',vehicle_speeds)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_fuelconsumption.ny',vehicle_fuelconsumptions)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_noise.ny',vehicle_noiseemissions)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_co2.ny',vehicle_co2emissions)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_waiting.ny',vehicle_waitingtimes)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_travel.ny',vehicle_traveltimes)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentspeeds.ny',agent_speeds)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentfuelconsumption.ny',agent_fuelconsumptions)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentnoise.ny',agent_noiseemissions)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentco2.ny',agent_co2emissions)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentwaiting.ny',agent_waitingtimes)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agenttravel.ny',agent_traveltimes)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foesspeeds.ny',foes_speeds)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foesfuelconsumption.ny',foes_fuelconsumptions)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foesnoise.ny',foes_noiseemissions)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foesco2.ny',foes_co2emissions)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foeswaiting.ny',foes_waitingtimes)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foestravel.ny',foes_traveltimes)
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedatafoesco2.ny',ftd['co2'])
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedatafoesnoise.ny',ftd['noise'])
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedatafoesfuel.ny',ftd['fuel'])
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedataagentco2.ny',atd['co2'])
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedataagentnoise.ny',atd['noise'])
            np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedataagentfuel.ny',atd['fuel'])
        
        sp_avg = np.mean(vehicle_speeds)
        fuel_avg = np.mean(vehicle_fuelconsumptions)
        noise_avg = np.mean(vehicle_noiseemissions)
        waiting_avg = np.mean(vehicle_waitingtimes)
        co2_avg = np.mean(vehicle_co2emissions)
        travel_avg = np.mean(vehicle_traveltimes)
        sp_std = np.std(vehicle_speeds)
        fuel_std = np.std(vehicle_fuelconsumptions)
        no_std = np.std(vehicle_noiseemissions)
        waiting_std = np.std(vehicle_waitingtimes)
        co2_std = np.std(vehicle_co2emissions)
        travel_std = np.std(vehicle_traveltimes)
        
        speedsplot.append(sp_avg)
        fuels.append(fuel_avg)
        noise.append(noise_avg)
        waitingtimes.append(waiting_avg)
        co2em.append(co2_avg)
        traveltimes.append(travel_avg)
        speedsplot_std.append(sp_std)
        fuels_std.append(fuel_std)
        noise_std.append(no_std)
        waitingtimes_std.append(waiting_std)
        co2em_std.append(co2_std)
        traveltimes_std.append(travel_std)
        x_axis.append(NUM_AGENTS)
        print('SPAWNED AGENTS || ARRIVED AGENTS')
        print(str(agents)+' || '+str(len(arrived)))
        print('SPEED TIME AVERAGE || SPEED SPACE AVERAGE || FLOW AVERAGE')
        print(str(sta)+' || '+str(ssa)+' || '+str(fa))
    else: 
        repeatsim = 3 if not USE_PARAM_GUI else int(params_s[1])
        start = 0
        step_perc = 1/max(numberofsim)
        for i in numberofsim:
            # if i==14:
            #     start = 1
            # else:
            #     start = 0
            run_speed = []
            run_fuel = []
            run_waiting = []
            run_noise = []
            run_co2 = []
            run_travel = []
            if not USE_NUM_AGENTS:
                NUM_AGENTS = step_perc*i*NUMBER_OF_CARS
            # start simulations and repeat them
            for j in range(start,repeatsim):
                ssize = 0.05
                if i == 0:
                    ssize = 1.0
                retval,agents,arrived,sta,ssa,fa,atd,ftd = single_sim(NUMBER_OF_CARS,1.0,True,TIME_HORIZON,ssize,True,True,12,step_perc*(i),mapdata,False,i,NUM_ALGS=NUM_ALGS,ONLINE=False,CONSIDER_WORKS=False)
                # retval,agents,arrived,sta,ssa,fa,atd,ftd = single_sim(NUMBER_OF_CARS,1.0,False,i,ssize,True,True,12,1,mapdata,False,i,NUM_ALGS=NUM_ALGS,ONLINE=True,CONSIDER_WORKS=False) # decomment this and comment the previous line for time analysis
                # prepare data structures for file saving
                vehicle_speeds = []
                vehicle_fuelconsumptions = []
                vehicle_waitingtimes = []
                vehicle_co2emissions = []
                vehicle_noiseemissions = []
                vehicle_traveltimes = []
                agent_speeds = []
                agent_fuelconsumptions = []
                agent_waitingtimes = []
                agent_co2emissions = []
                agent_noiseemissions = []
                agent_traveltimes = []
                foes_speeds = []
                foes_fuelconsumptions = []
                foes_waitingtimes = []
                foes_co2emissions = []
                foes_noiseemissions = []
                foes_traveltimes = []
                # retrieve data from the simulation
                for r in retval:
                    print(r)
                    if r[3]>0: # add values to data structures for global evaluation
                        vehicle_speeds.append(r[2])
                        vehicle_fuelconsumptions.append(r[4])
                        vehicle_waitingtimes.append(r[5])
                        vehicle_co2emissions.append(r[7])
                        vehicle_noiseemissions.append(r[6])
                        vehicle_traveltimes.append(r[8])
                        if r[0].__contains__('agent'): # add values to data structures for agent evaluation
                            agent_speeds.append(r[2])
                            agent_fuelconsumptions.append(r[4])
                            agent_waitingtimes.append(r[5])
                            agent_co2emissions.append(r[7])
                            agent_noiseemissions.append(r[6])
                            agent_traveltimes.append(r[8])
                        elif (not r[0].__contains__('random') and not r[0].__contains__('bus')): # add values to data structures for non-agent evaluation
                            foes_speeds.append(r[2])
                            foes_fuelconsumptions.append(r[4])
                            foes_waitingtimes.append(r[5])
                            foes_co2emissions.append(r[7])
                            foes_noiseemissions.append(r[6])
                            foes_traveltimes.append(r[8])
                run_speed.append(np.mean(vehicle_speeds))
                run_fuel.append(np.mean(vehicle_fuelconsumptions))
                run_noise.append(np.mean(vehicle_noiseemissions))
                run_waiting.append(np.mean(vehicle_waitingtimes))
                run_co2.append(np.mean(vehicle_co2emissions))
                run_travel.append(np.mean(vehicle_traveltimes))
                if SAVE_FILE: # save files
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_speeds.ny',vehicle_speeds)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_fuelconsumption.ny',vehicle_fuelconsumptions)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_noise.ny',vehicle_noiseemissions)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_co2.ny',vehicle_co2emissions)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_waiting.ny',vehicle_waitingtimes)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_travel.ny',vehicle_traveltimes)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentspeeds.ny',agent_speeds)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentfuelconsumption.ny',agent_fuelconsumptions)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentnoise.ny',agent_noiseemissions)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentco2.ny',agent_co2emissions)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agentwaiting.ny',agent_waitingtimes)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_agenttravel.ny',agent_traveltimes)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foesspeeds.ny',foes_speeds)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foesfuelconsumption.ny',foes_fuelconsumptions)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foesnoise.ny',foes_noiseemissions)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foesco2.ny',foes_co2emissions)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foeswaiting.ny',foes_waitingtimes)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_foestravel.ny',foes_traveltimes)
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedatafoesco2.ny',ftd['co2'])
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedatafoesnoise.ny',ftd['noise'])
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedatafoesfuel.ny',ftd['fuel'])
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedataagentco2.ny',atd['co2'])
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedataagentnoise.ny',atd['noise'])
                    np.save(IMG_FOLDER+"run"+str(i)+"_"+str(j)+"_"+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_timedataagentfuel.ny',atd['fuel'])
        
            sp_avg = np.mean(run_speed)
            fuel_avg = np.mean(run_fuel)
            noise_avg = np.mean(run_noise)
            waiting_avg = np.mean(run_waiting)
            co2_avg = np.mean(run_co2)
            travel_avg = np.mean(run_travel)
            sp_std = np.std(run_speed)
            fuel_std = np.std(run_fuel)
            no_std = np.std(run_noise)
            waiting_std = np.std(run_waiting)
            co2_std = np.std(run_co2)
            travel_std = np.std(run_travel)
            
            speedsplot.append(sp_avg)
            fuels.append(fuel_avg)
            noise.append(noise_avg)
            waitingtimes.append(waiting_avg)
            co2em.append(co2_avg)
            traveltimes.append(travel_avg)
            speedsplot_std.append(sp_std)
            fuels_std.append(fuel_std)
            noise_std.append(no_std)
            waitingtimes_std.append(waiting_std)
            co2em_std.append(co2_std)
            traveltimes_std.append(travel_std)
            x_axis.append(step_perc*(i)*NUMBER_OF_CARS)
            print('SPAWNED AGENTS || ARRIVED AGENTS')
            print(str(agents)+' || '+str(len(arrived)))
            print('SPEED TIME AVERAGE || SPEED SPACE AVERAGE || FLOW AVERAGE')
            print(str(sta)+' || '+str(ssa)+' || '+str(fa))
