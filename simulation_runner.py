HIL = False
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

CREATE_BEHAVIOURS = False
RUN_SIMULATION = True
NUM_ALGS = 4
IMG_FOLDER = 'SalernoScenario_12h_incident/'
# IMG_FOLDER = 'data4plots/testdata5/'
SCENARIO = 'Unisa'
NUMBER_OF_CARS = 20
NUM_AGENTS = 1
USE_NUM_AGENTS = True
PERC_AGENTS = 0.3
TIME_HORIZON = 3 # keep around 5
SAVE_IMG = False
SAVE_FILE = False

USE_PARAM_GUI = True

if CREATE_BEHAVIOURS:
    create_behaviours(NUM_ALGS,MapData(SCENARIO),False)
    create_paths(NUM_ALGS,MapData(SCENARIO),False)

if RUN_SIMULATION:
    params_s = []
    if USE_PARAM_GUI:
        gui_create_and_run()
        f = open('sim_config.txt','r')
        params = f.readline()
        f.close()
        params_s = params.split(';')
        NUMBER_OF_CARS = int(params_s[0])
    # Arguments:
    # maximum number of vehicles, percentage of uni-related cars, show GUI, time horizon, simulation step size, include bus, include random cars,
    # starting hour, percentage of agent cars among the uni-related ones, use desired number of agents (instead of percentage), number of agents
    # SIMULATION ENDS WHEN ALL AGENTS HAVE ARRIVED
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
    # maxrange = 21 if NUMBER_OF_CARS>=20 else 11
    maxrange = 21
    # numberofsim = range(1,maxrange) if not HIL else [1]
    numberofsim = [1]
    mapdata = MapData(SCENARIO)
    if len(numberofsim)<2:
        i = 0
        j = 0
        if not USE_NUM_AGENTS:
            NUM_AGENTS = PERC_AGENTS*NUMBER_OF_CARS
        retval,agents,arrived,sta,ssa,fa,atd,ftd = single_sim(NUMBER_OF_CARS,1.0,True,TIME_HORIZON,0.05,True,True,12,PERC_AGENTS,mapdata,USE_NUM_AGENTS,NUM_AGENTS,NUM_ALGS=NUM_ALGS,ONLINE=True,CONSIDER_WORKS=False)
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
        for r in retval:
            print(r)
            if r[3]>0:
                vehicle_speeds.append(r[2])
                vehicle_fuelconsumptions.append(r[4])
                vehicle_waitingtimes.append(r[5])
                vehicle_co2emissions.append(r[7])
                vehicle_noiseemissions.append(r[6])
                vehicle_traveltimes.append(r[8])
                if r[0].__contains__('agent'):
                    agent_speeds.append(r[2])
                    agent_fuelconsumptions.append(r[4])
                    agent_waitingtimes.append(r[5])
                    agent_co2emissions.append(r[7])
                    agent_noiseemissions.append(r[6])
                    agent_traveltimes.append(r[8])
                else:
                    foes_speeds.append(r[2])
                    foes_fuelconsumptions.append(r[4])
                    foes_waitingtimes.append(r[5])
                    foes_co2emissions.append(r[7])
                    foes_noiseemissions.append(r[6])
                    foes_traveltimes.append(r[8])
        if SAVE_FILE:
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
        # step_perc = 1.0
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
            for j in range(start,repeatsim):
                ssize = 0.05
                if i == 0:
                    ssize = 1.0
                retval,agents,arrived,sta,ssa,fa,atd,ftd = single_sim(NUMBER_OF_CARS,1.0,True,TIME_HORIZON,ssize,True,True,12,step_perc*(i),mapdata,False,i,NUM_ALGS=NUM_ALGS,ONLINE=False,CONSIDER_WORKS=False)
                # retval,agents,arrived,sta,ssa,fa,atd,ftd = single_sim(NUMBER_OF_CARS,1.0,False,i,ssize,True,True,12,1,mapdata,False,i,NUM_ALGS=NUM_ALGS,ONLINE=True,CONSIDER_WORKS=False)
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
                for r in retval:
                    print(r)
                    if r[3]>0:
                        vehicle_speeds.append(r[2])
                        vehicle_fuelconsumptions.append(r[4])
                        vehicle_waitingtimes.append(r[5])
                        vehicle_co2emissions.append(r[7])
                        vehicle_noiseemissions.append(r[6])
                        vehicle_traveltimes.append(r[8])
                        if r[0].__contains__('agent'):
                            agent_speeds.append(r[2])
                            agent_fuelconsumptions.append(r[4])
                            agent_waitingtimes.append(r[5])
                            agent_co2emissions.append(r[7])
                            agent_noiseemissions.append(r[6])
                            agent_traveltimes.append(r[8])
                        elif (not r[0].__contains__('random') and not r[0].__contains__('bus')):
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
                if SAVE_FILE:
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
            
    # elaborate_and_make_plots()
    # if SAVE_FILE:
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_speeds.ny',speedsplot)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_fuelconsumption.ny',fuels)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_noise.ny',noise)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_co2.ny',co2em)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_waiting.ny',waitingtimes)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_travel.ny',traveltimes)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_speeds_std.ny',speedsplot_std)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_fuelconsumption_std.ny',fuels_std)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_noise_std.ny',noise_std)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_co2_std.ny',co2em_std)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_waiting_std.ny',waitingtimes_std)
    #     np.save(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(NUM_AGENTS)+'_agents_travel_std.ny',traveltimes_std)
    
    # if SAVE_IMG:
    #     plotter.plotter(x_axis,speedsplot,speedsplot_std,'Average speed',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
    #     plotter.plotter(x_axis,fuels,fuels_std,'Fuel consumption',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
    #     plotter.plotter(x_axis,waitingtimes,waitingtimes_std,'Waiting times',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
    #     plotter.plotter(x_axis,noise,noise_std,'Noise emissions',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
    #     plotter.plotter(x_axis,co2em,co2em_std,'CO2 emissions',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
    #     plotter.plotter(x_axis,traveltimes,traveltimes_std,'Travel times',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
        # plt.style.use('ggplot')
        # plt.plot(speedsplot)
        # plt.title('Average speed and controlled cars')
        # plt.xlabel('Number of controlled cars')
        # plt.ylabel('Average speed')
        # plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_speeds.png')
        # plt.show()
        # plt.plot(fuels)
        # plt.title('Fuel consumption and controlled cars')
        # plt.xlabel('Number of controlled cars')
        # plt.ylabel('Average fuel consumption')
        # plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_fuelconsumption.png')
        # plt.show()
        # plt.plot(waitingtimes)
        # plt.title('Waiting time and controlled cars')
        # plt.xlabel('Number of controlled cars')
        # plt.ylabel('Average waiting time')
        # plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_waitingtime.png')
        # plt.show()
        # plt.plot(co2em)
        # plt.title('CO2 emissions and controlled cars')
        # plt.xlabel('Number of controlled cars')
        # plt.ylabel('Average CO2 emission')
        # plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_co2.png')
        # plt.show()
        # plt.plot(noise)
        # plt.title('Noise emission and controlled cars')
        # plt.xlabel('Number of controlled cars')
        # plt.ylabel('Average noise emissions')
        # plt.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_noise.png')
        # plt.show()
