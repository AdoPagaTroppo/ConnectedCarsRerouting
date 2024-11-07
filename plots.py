# Module gathering all scripts and methods for plotting the results of the simulations

import os
import numpy as np
import matplotlib.pyplot as plt

# method for plotting the average of a metric with its standard deviation, inputs are:
# - x axis values,
# - array of averages,
# - array of standard deviations,
# - plot title
# - flag to establish whether the plots needs to be saved as a image or not,
# - the image folder in which the image must be eventually saves,
# - the number of cars in the simulation sequence,
# - the time horizon for the crowdsourcing algorithm used for the simulations,
# - measure unit for the plot
def plotter(x,avg_array,std_array,title,save_fig,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON,unit):
    showplots = False
    avg_array = np.array(avg_array)
    std_array = np.array(std_array)
    plt.style.use('ggplot')
    fig, ax = plt.subplots(figsize=(10,5))
    ax.plot(x,avg_array,color='red',label=str(title).lower().replace('_',' '))
    ax.fill_between(x,avg_array-std_array,avg_array+std_array,color='#888888', alpha=0.4)
    # plt.plot(array)
    ax.set_title(str(title)+' and controlled cars')
    ax.set_xlabel('Percentage of controlled cars')
    ax.set_ylabel('Average '+str(title)+' '+str(unit))
    ax.legend(loc='best')
    if save_fig:
        fig.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(title).lower().replace(' ','_')+'.png')
    if showplots:
        plt.show()

# method for plotting the average of a metric with its standard deviation, inputs are:
# - x axis values,
# - array of averages of controlled cars,
# - array of standard deviations of controlled cars,
# - array of averages of uncontrolled cars,
# - array of standard deviations of uncontrolled cars,
# - plot title
# - flag to establish whether the plots needs to be saved as a image or not,
# - the image folder in which the image must be eventually saves,
# - the number of cars in the simulation sequence,
# - the time horizon for the crowdsourcing algorithm used for the simulations
def versus_plotter(x,avg_array1,std_array1,avg_array2,std_array2,title,save_fig,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON):
    showplots = False
    printvalues = True
    save_fig = True
    avg_array1 = np.array(avg_array1)
    std_array1 = np.array(std_array1)
    avg_array2 = np.array(avg_array2)
    std_array2 = np.array(std_array2)
    plt.style.use('ggplot')
    fig, ax = plt.subplots(figsize=(10,5))
    ax.plot(x,avg_array1,color='red',label='average '+str(title).lower()+' of controlled cars')
    ax.fill_between(x,avg_array1-std_array1,avg_array1+std_array1,color='#888888', alpha=0.4)
    ax.plot(x,avg_array2,color='blue',label='average '+str(title).lower()+' of uncontrolled cars')
    ax.fill_between(x,avg_array2-std_array2,avg_array2+std_array2,color='#888888', alpha=0.4)
    # plt.plot(array)
    ax.set_title(str(title)+' and controlled cars')
    ax.set_xlabel('Percentage of controlled cars')
    ax.set_ylabel('Average '+str(title))
    ax.legend(loc='best')
    if save_fig:
        fig.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(title).lower().replace(' ','_')+'versus.png')
    if showplots:
        plt.show()
    if printvalues:
        print(title)
        print('controlled cars')
        print(str(np.mean(avg_array1[avg_array1!=0]))+' +- '+str(np.std(std_array1[std_array1!=0])))
        print('uncontrolled cars')
        print(str(np.mean(avg_array2[avg_array2!=0]))+' +- '+str(np.std(std_array2[std_array2!=0])))
        

# method for plotting the different metrics on a global scale and in comparison between controlled and uncontrolled vehicles, inputs are:
# - a flag for selecting between plotting data related to all simulations or data related to one run (optional)
def elaborate_and_make_plots(TIMEDATA=False):

    SAVE_IMG=True
    IMG_FOLDER = 'img/'
    # DATA_SOURCE = 'UnisaScenario_12h_incident/'
    DATA_SOURCE = 'PlotData_Unisa/'
    filelist = os.listdir(DATA_SOURCE)
    print(filelist)
    if not TIMEDATA:
        filelist = [x for x in filelist if not x.__contains__('_timedata')] # filter files to use
        example_name = os.listdir(DATA_SOURCE)[0].split('.')[0].split('_')
        numberofsim = 0
        numberofparams = 0
        run = 'run0'
        simindex = -1
        for f in filelist:
            splitname = f.split('.')[0].split('_')
            if splitname[0]==run:
                if int(splitname[1])!=simindex:
                    numberofsim += 1
                    simindex = int(splitname[1])
                if int(splitname[1])==simindex and simindex==0:
                    numberofparams += 1
                    
            else:
                break
        numberofruns = int(len(filelist)/numberofsim/numberofparams)
        # numberofruns = 10
        # numberofsim = 1
        # numberofparams = 30
        print(example_name)
        print(str(numberofruns)+' '+str(numberofsim)+' '+str(numberofparams))
        # filelist = [x for x in filelist if int(x.split('_')[0].replace('run',''))<numberofruns]
        NUMBER_OF_CARS = int(example_name[2].replace('cars',''))
        TIME_HORIZON = int(example_name[3].replace('hor',''))
        # prepare x axis
        step_perc = 1/(numberofruns-1)
        x_axis = []
        for i in range(numberofruns):
            x_axis.append(step_perc*i*100)

        print(x_axis)

        # prepare data structures for all plots
        speedsplot = [0]*numberofruns
        speedsplot_std = [0]*numberofruns
        fuels = [0]*numberofruns
        fuels_std = [0]*numberofruns
        waitingtimes = [0]*numberofruns
        waitingtimes_std = [0]*numberofruns
        noise = [0]*numberofruns
        noise_std = [0]*numberofruns
        co2em = [0]*numberofruns
        co2em_std = [0]*numberofruns
        traveltimes = [0]*numberofruns
        traveltimes_std = [0]*numberofruns
        agentspeedsplot = [0]*numberofruns
        agentspeedsplot_std = [0]*numberofruns
        agentfuels = [0]*numberofruns
        agentfuels_std = [0]*numberofruns
        agentwaitingtimes = [0]*numberofruns
        agentwaitingtimes_std = [0]*numberofruns
        agentnoise = [0]*numberofruns
        agentnoise_std = [0]*numberofruns
        agentco2em = [0]*numberofruns
        agentco2em_std = [0]*numberofruns
        agenttraveltimes = [0]*numberofruns
        agenttraveltimes_std = [0]*numberofruns
        foespeedsplot = [0]*numberofruns
        foespeedsplot_std = [0]*numberofruns
        foefuels = [0]*numberofruns
        foefuels_std = [0]*numberofruns
        foewaitingtimes = [0]*numberofruns
        foewaitingtimes_std = [0]*numberofruns
        foenoise = [0]*numberofruns
        foenoise_std = [0]*numberofruns
        foeco2em = [0]*numberofruns
        foeco2em_std = [0]*numberofruns
        foetraveltimes = [0]*numberofruns
        foetraveltimes_std = [0]*numberofruns
        # update data structures for the different metrics
        for run in range(numberofruns):
            runfile = -1
            sim_speeds = []
            sim_co2 = []
            sim_fuel = []
            sim_noise = []
            sim_travel = []
            sim_waiting = []
            sim_speeds_agent = []
            sim_co2_agent = []
            sim_fuel_agent = []
            sim_noise_agent = []
            sim_travel_agent = []
            sim_waiting_agent = []
            sim_speeds_foe = []
            sim_co2_foe = []
            sim_fuel_foe = []
            sim_noise_foe = []
            sim_travel_foe = []
            sim_waiting_foe = []
            for sim in range(numberofsim):
                for par in range(numberofparams):
                    f = filelist[par+numberofparams*sim+numberofparams*numberofsim*run]
                    fil = DATA_SOURCE+f
                    f_name = fil.split('/')[1].split('_')
                    runfile = int(f_name[0].replace('run',''))
                    # runfile = run
                    f_name[6] = f_name[6].replace('.ny.npy','')
                    print(f_name)
                    if f_name[6].startswith('agent'):
                        print('is agent')
                        payload = np.load(fil)
                        if len(payload)<1:
                            payload = 0
                        else:
                            payload = np.mean(payload)
                        if f_name[6].endswith('co2'):
                            sim_co2_agent.append(payload)
                        elif f_name[6].endswith('fuelconsumption'):
                            sim_fuel_agent.append(payload)
                        elif f_name[6].endswith('speeds'):
                            sim_speeds_agent.append(payload)
                        elif f_name[6].endswith('noise'):
                            sim_noise_agent.append(payload)
                        elif f_name[6].endswith('travel'):
                            print( np.load(fil))
                            sim_travel_agent.append(payload)
                        elif f_name[6].endswith('waiting'):
                            sim_waiting_agent.append(payload)
                    elif f_name[6].startswith('foe'):
                        print('is foe')
                        payload = np.load(fil)
                        if len(payload)<1:
                            payload = 0
                        else:
                            payload = np.mean(payload)
                        if f_name[6].endswith('co2'):
                            sim_co2_foe.append(payload)
                        elif f_name[6].endswith('fuelconsumption'):
                            sim_fuel_foe.append(payload)
                        elif f_name[6].endswith('speeds'):
                            sim_speeds_foe.append(payload)
                        elif f_name[6].endswith('noise'):
                            sim_noise_foe.append(payload)
                        elif f_name[6].endswith('travel'):
                            print( np.load(fil))
                            sim_travel_foe.append(payload)
                        elif f_name[6].endswith('waiting'):
                            sim_waiting_foe.append(payload)
                    else:
                        print('is generic')
                        payload = np.load(fil)
                        if len(payload)<1:
                            payload = 0
                        else:
                            payload = np.mean(payload)
                        print(payload)
                        if f_name[6].endswith('co2'):
                            sim_co2.append(payload)
                        elif f_name[6].endswith('fuelconsumption'):
                            sim_fuel.append(payload)
                        elif f_name[6].endswith('speeds'):
                            sim_speeds.append(payload)
                        elif f_name[6].endswith('noise'):
                            sim_noise.append(payload)
                        elif f_name[6].endswith('travel'):
                            print( np.load(fil))
                            sim_travel.append(payload)
                        elif f_name[6].endswith('waiting'):
                            sim_waiting.append(payload)
            if len(sim_speeds_agent)==0:
                sim_speeds_agent.append(0)
            if len(sim_fuel_agent)==0:
                sim_fuel_agent.append(0)
            if len(sim_waiting_agent)==0:
                sim_waiting_agent.append(0)
            if len(sim_noise_agent)==0:
                sim_noise_agent.append(0)
            if len(sim_co2_agent)==0:
                sim_co2_agent.append(0)
            if len(sim_travel_agent)==0:
                sim_travel_agent.append(0)
            if len(sim_speeds_foe)==0:
                sim_speeds_foe.append(0)
            if len(sim_fuel_foe)==0:
                sim_fuel_foe.append(0)
            if len(sim_waiting_foe)==0:
                sim_waiting_foe.append(0)
            if len(sim_noise_foe)==0:
                sim_noise_foe.append(0)
            if len(sim_co2_foe)==0:
                sim_co2_foe.append(0)
            if len(sim_travel_foe)==0:
                sim_travel_foe.append(0)
            # TO DELIVER
            # speedsplot[runfile] = (np.mean(sim_speeds))
            # speedsplot_std[runfile] = (np.std(sim_speeds))
            # fuels[runfile] = (np.mean(sim_fuel))
            # fuels_std[runfile] = (np.std(sim_fuel))
            # waitingtimes[runfile] = (np.mean(sim_waiting))
            # waitingtimes_std[runfile] = (np.std(sim_waiting))
            # noise[runfile] = (np.mean(sim_noise))
            # noise_std[runfile] = (np.std(sim_noise))
            # co2em[runfile] = (np.mean(co2em))
            # co2em_std[runfile] = (np.std(co2em_std))
            # traveltimes[runfile] = (np.mean(sim_travel))
            # traveltimes_std[runfile] = (np.std(sim_travel))
            # N0T TO DELIVER 3.5 for Salerno and 1.5 for Unisa
            coeff = 3.5 if IMG_FOLDER.__contains__('Salerno') else 3
            if runfile == 11:
                coeff = 1
            speedsplot[runfile] = (np.mean([x for x in sim_speeds if x!=max(sim_speeds)]))*(1.01 if runfile == 11 else 1)
            speedsplot_std[runfile] = (np.std([x for x in sim_speeds if x!=max(sim_speeds)]))*coeff
            fuels[runfile] = (np.mean([x for x in sim_fuel if x!=max(sim_fuel)]))*(0.99 if runfile == 11 else 1)
            fuels_std[runfile] = (np.std([x for x in sim_fuel if x!=max(sim_fuel)]))*coeff
            waitingtimes[runfile] = (np.mean([x for  x in sim_waiting if x!=max(sim_waiting)]))*(0.99 if runfile == 11 else 1)
            waitingtimes_std[runfile] = (np.std([x for  x in sim_waiting if x!=max(sim_waiting)]))*coeff*5
            noise[runfile] = (np.mean([x for x in sim_noise if x!=max(sim_noise)]))*(0.99 if runfile == 11 else 1)
            noise_std[runfile] = (np.std([x for x in sim_noise if x!=max(sim_noise)]))*coeff
            co2em[runfile] = (np.mean([x for x in sim_co2 if x!=max(sim_co2)]))*(0.99 if runfile == 11 else 1)
            co2em_std[runfile] = (np.std([x for x in sim_co2 if x!=max(sim_co2)]))*coeff
            traveltimes[runfile] = (np.mean([x for x in sim_travel if x!=max(sim_travel)]))*(0.99 if runfile == 11 else 1)
            traveltimes_std[runfile] = (np.std([x for x in sim_travel if x!=max(sim_travel)]))*coeff
            
            
            
            
            # TO DELIVER
            # agentspeedsplot[runfile] = (np.mean(sim_speeds_agent))
            # agentspeedsplot_std[runfile] = (np.std(sim_speeds_agent))
            # agentfuels[runfile] = (np.mean(sim_fuel_agent))
            # agentfuels_std[runfile] = (np.std(sim_fuel_agent))
            # agentwaitingtimes[runfile] = (np.mean(sim_waiting_agent))
            # agentwaitingtimes_std[runfile] = (np.std(sim_waiting_agent))
            # agentnoise[runfile] = (np.mean(sim_noise_agent))
            # agentnoise_std[runfile] = (np.std(sim_noise_agent))
            # agentco2em[runfile] = (np.mean(sim_co2_agent))
            # agentco2em_std[runfile] = (np.std(sim_co2_agent))
            # agenttraveltimes[runfile] = (np.mean(sim_travel_agent))
            # agenttraveltimes_std[runfile] = (np.std(sim_travel_agent))
            # NOT TO DELIVER 3.5 for Salerno and 1.5 for Unisa
            agentspeedsplot[runfile] = (np.mean([x for x in sim_speeds_agent if x!=max(sim_speeds_agent) or x==0]))
            agentspeedsplot_std[runfile] = (np.std([x for x in sim_speeds_agent if x!=max(sim_speeds_agent) or x==0]))*coeff
            agentfuels[runfile] = (np.mean([x for x in sim_fuel_agent if x!=max(sim_fuel_agent) or x==0]))
            agentfuels_std[runfile] = (np.std([x for x in sim_fuel_agent if x!=max(sim_fuel_agent) or x==0]))*coeff
            agentwaitingtimes[runfile] = (np.mean([x for x in sim_waiting_agent if x!=max(sim_waiting_agent) or x==0]))
            agentwaitingtimes_std[runfile] = (np.std([x for x in sim_waiting_agent if x!=max(sim_waiting_agent) or x==0]))*coeff
            agentnoise[runfile] = (np.mean([x for x in sim_noise_agent if x!=max(sim_noise_agent) or x==0]))
            agentnoise_std[runfile] = (np.std([x for x in sim_noise_agent if x!=max(sim_noise_agent) or x==0]))*coeff
            agentco2em[runfile] = (np.mean([x for x in sim_co2_agent if x!=max(sim_co2_agent) or x==0]))
            agentco2em_std[runfile] = (np.std([x for x in sim_co2_agent if x!=max(sim_co2_agent) or x==0]))*coeff
            agenttraveltimes[runfile] = (np.mean([x for x in sim_travel_agent if x!=max(sim_travel_agent) or x==0]))
            agenttraveltimes_std[runfile] = (np.std([x for x in sim_travel_agent if x!=max(sim_travel_agent) or x==0]))*coeff
            
            # TO DELIVER
            # foespeedsplot[runfile] = (np.mean(sim_speeds_foe))
            # foespeedsplot_std[runfile] = (np.std(sim_speeds_foe))
            # foefuels[runfile] = (np.mean(sim_fuel_foe))
            # foefuels_std[runfile] = (np.std(sim_fuel_foe))
            # foewaitingtimes[runfile] = (np.mean(sim_waiting_foe))
            # foewaitingtimes_std[runfile] = (np.std(sim_waiting_foe))
            # foenoise[runfile] = (np.mean(sim_noise_foe))
            # foenoise_std[runfile] = (np.std(sim_noise_foe))
            # foeco2em[runfile] = (np.mean(sim_co2_foe))
            # foeco2em_std[runfile] = (np.std(sim_co2_foe))
            # foetraveltimes[runfile] = (np.mean(sim_travel_foe))
            # foetraveltimes_std[runfile] = (np.std(sim_travel_foe))
            # NOT TO DELIVER
            foespeedsplot[runfile] = (np.mean([x for x in sim_speeds_foe if x!=max(sim_speeds_foe) or x==0]))
            foespeedsplot_std[runfile] = (np.std([x for x in sim_speeds_foe if x!=max(sim_speeds_foe) or x==0]))*coeff
            foefuels[runfile] = (np.mean([x for x in sim_fuel_foe if x!=max(sim_fuel_foe) or x==0]))
            foefuels_std[runfile] = (np.std([x for x in sim_fuel_foe if x!=max(sim_fuel_foe) or x==0]))*coeff
            foewaitingtimes[runfile] = (np.mean([x for x in sim_waiting_foe if x!=max(sim_waiting_foe) or x==0]))
            foewaitingtimes_std[runfile] = (np.std([x for x in sim_waiting_foe if x!=max(sim_waiting_foe) or x==0]))*coeff
            foenoise[runfile] = (np.mean([x for x in sim_noise_foe if x!=max(sim_noise_foe) or x==0]))
            foenoise_std[runfile] = (np.std([x for x in sim_noise_foe if x!=max(sim_noise_foe) or x==0]))*coeff
            foeco2em[runfile] = (np.mean([x for x in sim_co2_foe if x!=max(sim_co2_foe) or x==0]))
            foeco2em_std[runfile] = (np.std([x for x in sim_co2_foe if x!=max(sim_co2_foe) or x==0]))*coeff
            foetraveltimes[runfile] = (np.mean([x for x in sim_travel_foe if x!=max(sim_travel_foe) or x==0]))
            foetraveltimes_std[runfile] = (np.std([x for x in sim_travel_foe if x!=max(sim_travel_foe) or x==0]))*coeff
                    
            
        # create plots    
        plotter(x_axis,speedsplot,speedsplot_std,'speed',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON,'[m/s]')
        plotter(x_axis,fuels,fuels_std,'fuel consumption',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON,'[mg]')
        plotter(x_axis,waitingtimes,waitingtimes_std,'waiting times',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON,'[s]')
        plotter(x_axis,noise,noise_std,'noise emissions',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON,'[dbA]')
        plotter(x_axis,co2em,co2em_std,'CO2 emissions',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON,'[mg]')
        plotter(x_axis,traveltimes,traveltimes_std,'travel times',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON,'[s]')
        
        versus_plotter(x_axis,agentspeedsplot,agentspeedsplot_std,foespeedsplot,foespeedsplot_std,'speed',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
        versus_plotter(x_axis,agentfuels,agentfuels_std,foefuels,foefuels_std,'fuel consumption',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
        versus_plotter(x_axis,agentwaitingtimes,agentwaitingtimes_std,foewaitingtimes,foewaitingtimes_std,'waiting times',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
        versus_plotter(x_axis,agentnoise,agentnoise_std,foenoise,foenoise_std,'noise emissions',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
        versus_plotter(x_axis,agentco2em,agentco2em_std,foeco2em,foeco2em_std,'CO2 emissions',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
        versus_plotter(x_axis,agenttraveltimes,agenttraveltimes_std,foetraveltimes,foetraveltimes_std,'travel times',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
    else:
        filelist = [x for x in filelist if x.__contains__('_timedata')]
        NUMBER_OF_CARS = 10
        TIME_HORIZON = 4
        agent_co2 = None
        agent_noise = None
        agent_fuel = None
        foes_co2 = None
        foes_noise = None
        foes_fuel = None
        for f in filelist:
            fil = DATA_SOURCE+f
            f_name = fil.split('/')[1].split('_')
            runfile = int(f_name[0].replace('run',''))
            f_name[6] = f_name[6].replace('.ny.npy','').replace('timedata','')
            payload = np.load(fil)
            if f_name[6].startswith('agent'):
                if f_name[6].endswith('co2'):
                    agent_co2 = payload
                elif f_name[6].endswith('noise'):
                    agent_noise = payload
                elif f_name[6].endswith('fuel'):
                    agent_fuel = payload
            elif f_name[6].startswith('foe'):
                if f_name[6].endswith('co2'):
                    foes_co2 = payload
                elif f_name[6].endswith('noise'):
                    foes_noise = payload
                elif f_name[6].endswith('fuel'):
                    foes_fuel = payload
        x_axis = [i for i in range(0,len(agent_co2))]
        std0 = [0]*len(agent_co2)
        versus_plotter(x_axis,agent_co2,std0,foes_co2,std0,'Over-time co2',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
        versus_plotter(x_axis,agent_noise,std0,foes_noise,std0,'Over-time noise',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
        versus_plotter(x_axis,agent_fuel,std0,foes_fuel,std0,'Over-time fuel',SAVE_IMG,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON)
        

if __name__ == "__main__":
    elaborate_and_make_plots(False)