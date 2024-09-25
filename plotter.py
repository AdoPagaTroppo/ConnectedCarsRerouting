import matplotlib.pyplot as plt
import numpy as np

def plotter(x,avg_array,std_array,title,save_fig,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON,unit):
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
    # plt.show()
    
def versus_plotter(x,avg_array1,std_array1,avg_array2,std_array2,title,save_fig,IMG_FOLDER,NUMBER_OF_CARS,TIME_HORIZON):
    avg_array1 = np.array(avg_array1)
    std_array1 = np.array(std_array1)
    avg_array2 = np.array(avg_array2)
    std_array2 = np.array(std_array2)
    plt.style.use('ggplot')
    fig, ax = plt.subplots(figsize=(10,5))
    ax.plot(x,avg_array1,color='red',label='average '+str(title).lower().replace(' ','_')+' of controlled cars')
    ax.fill_between(x,avg_array1-std_array1,avg_array1+std_array1,color='#888888', alpha=0.4)
    ax.plot(x,avg_array2,color='blue',label='average '+str(title).lower().replace(' ','_')+' of uncontrolled cars')
    ax.fill_between(x,avg_array2-std_array2,avg_array2+std_array2,color='#888888', alpha=0.4)
    # plt.plot(array)
    ax.set_title(str(title)+' and controlled cars')
    ax.set_xlabel('Number of controlled cars')
    ax.set_ylabel('Average '+str(title))
    ax.legend(loc='best')
    if save_fig:
        fig.savefig(IMG_FOLDER+str(NUMBER_OF_CARS)+'cars_'+str(TIME_HORIZON)+'hor_'+str(title).lower().replace(' ','_')+'versus.png')
    # plt.show()