import argparse
import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--input_filename',
                    type=str,
                    help='Debug file output by pid.cpp.')
FILENAME = parser.parse_args().input_filename
data = np.genfromtxt(FILENAME, delimiter='\t', names=True) 

p_errors = data['P_ERROR']
i_errors = data['I_ERROR']
d_errors = data['D_ERROR']
ctes = data['CTE']

assert len(ctes) == len(p_errors) == len(i_errors) == len(d_errors)
steps = np.arange(len(ctes))

p_patch = mpatches.Patch(color='red', label='P error')
i_patch = mpatches.Patch(color='blue', label='I error')
d_patch = mpatches.Patch(color='green', label='D error')
cte_patch= mpatches.Patch(color='orange', label='CTE')

plt.legend(handles=[p_patch, i_patch, d_patch, cte_patch])
plt.xlabel('KP=%.4f, KI=%.4f, KD=%.4f' % (data['KP'][0], data['KI'][0], data['KD'][0]))

plt.plot(steps, p_errors, color='red')
plt.plot(steps, i_errors, color='blue')
plt.plot(steps, d_errors, color='green')
plt.plot(steps, ctes, color='orange')

output_filename = FILENAME.split('.')[0] + '_plot.png'
plt.savefig(output_filename)
print('Saved plot at %s' % output_filename)
