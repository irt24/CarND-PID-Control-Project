import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

FILENAME = '/tmp/pid_wrapper_debug_twiddle.txt' 
data = np.genfromtxt(FILENAME, delimiter='\t', names=True) 

global_step = data['GLOBAL_STEP']

# Make plot colors consistent with process_pid_debug.py.
for item, color in [('KP', 'red'),
                    ('KI', 'blue'),
                    ('KD', 'green'),
                    ('BEST_ERROR_IN_CYCLE', 'orange')]:
  plt.clf()

  xs = global_step 
  ys = data[item]

  # BEST_ERROR is 0.0 at the beginning of the algorithm,
  # before it's actually computed. Remove those 0s from the list.
  if item == 'BEST_ERROR_IN_CYCLE':
    ys = [v for v in ys if v > 0]
    xs = xs[:len(ys)]

  plt.plot(xs, ys, color=color)
  plt.xlabel('Global step')
  plt.ylabel(item)
  plt.title('Evolution of %s in the Twiddle algorithm.' % item)
  output_filename = '%s_%s_plot.png' % (FILENAME.split('.')[0], item)
  plt.savefig(output_filename)
  print('Saved %s plot at %s' % (item, output_filename))

  if item == 'BEST_ERROR_IN_CYCLE':
    print('Min best error: ', min(ys))
    print('Max best error: ', max(ys))
