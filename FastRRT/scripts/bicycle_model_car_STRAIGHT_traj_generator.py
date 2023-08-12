import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
L=2.79
v_min=0
v_max=12
delta_dot_min=-0.2183
delta_dot_max=0.2183
delta_min=-0.5236
delta_max=0.5236
T=5#final time
N=25 # no. of collocation points
t_d=np.linspace(0,T,N) # discrete time steps


initial_vals=None
final_vals=None

def objective(var):
  h_k=t_d[1]-t_d[0]
  delta_dot=var[4*N:5*N]
  v=var[5*N:6*N]
  theta=var[2*N:3*N]
  sum1=0
  sum2=0
  sum3=0
  for i in range(N-1):

        sum1+=(h_k/2)*((delta_dot[i])**2 *(delta_dot[i+1])**2)
        sum2+=(h_k/2)*((v[i])**2 *(v[i+1])**2)
        sum3+=(h_k/2)*((theta[i])**2 *(theta[i+1])**2)
  sum=sum1+sum2

  return sum


def constraints(var):
 global initial_vals,final_vals
 x,y,theta,delta,delta_dot,v=np.split(var,6)


 h_k=t_d[3]-t_d[2]
 cons=np.zeros((4*(N-1)+12,))
 cons[:N-1]=x[1:]-x[:-1]-(h_k/2)*(v[1:]*np.cos(theta[1:])+v[:-1]*np.cos(theta[:-1]))
 cons[N-1:2*(N-1)]=y[1:]-y[:-1]-(h_k/2)*(v[1:]*np.sin(theta[1:])+v[:-1]*np.sin(theta[:-1]))
 cons[2*(N-1):3*(N-1)]=theta[1:]-theta[:-1]-(h_k/2)*(v[1:]*np.tan(delta[1:])+v[:-1]*np.tan(delta[:-1]))/L
 cons[3*(N-1):4*(N-1)]=delta[1:]-delta[:-1]-(h_k/2)*(delta_dot[1:]+delta_dot[:-1])
 cons[4*(N-1)] = x[0]-initial_vals[0]
 cons[4*(N-1) + 1] = y[0]-initial_vals[1]
 cons[4*(N-1) + 2] = theta[0]-initial_vals[2]
 cons[4*(N-1) + 3] = delta[0]-initial_vals[3]

 cons[4*(N-1) + 4]=x[N-1]-final_vals[0]
 cons[4*(N-1) + 5]=y[N-1]-final_vals[1]
 cons[4*(N-1) + 6]=theta[N-1]-final_vals[2]
 cons[4*(N-1) + 7] = delta[N-1]-final_vals[3]

 cons[4*(N-1) + 8]=v[0]-initial_vals[4]
 cons[4*(N-1) + 9]=delta_dot[0]-initial_vals[5]
 cons[4*(N-1) + 10]=v[N-1]-final_vals[4]
 cons[4*(N-1) + 11] =delta_dot[0]-final_vals[5]


 return cons



def get_trajectory(initial_values,final_values,initial_guess):
  global initial_vals,final_vals
  initial_vals=initial_values
  final_vals=final_values

  bounds = [(None, None)] * N + [(None, None)] * N + [(None, None)] * N + [(delta_min, delta_max)] * N + [(delta_dot_min, delta_dot_max)] * N +[(v_min, v_max)] * N



  constraint_eq = {'type': 'eq', 'fun': constraints}
  result = minimize(objective, initial_guess, method='SLSQP', constraints=constraint_eq, bounds=bounds)

  x_opt=result.x[:N]
  y_opt=result.x[N:2*N]
  theta_opt=result.x[2*N:3*N]
  delta_opt=result.x[3*N:4*N]
  delta_dot_opt=result.x[4*N:5*N]
  v_opt=result.x[5*N:6*N]

  return (x_opt,y_opt,theta_opt,delta_opt,delta_dot_opt,v_opt,t_d)


def get_interpolated_trajectory(x_opt,y_opt,theta_opt,delta_opt,delta_dot_opt,v_opt):




            t_c=np.linspace(0,T,50)
            x_opt_c=np.zeros_like(t_c)
            y_opt_c=np.zeros_like(t_c)
            theta_opt_c=np.zeros_like(t_c)
            delta_opt_c=np.zeros_like(t_c)
            delta_dot_opt_c=np.zeros_like(t_c)
            v_opt_c=np.zeros_like(t_c)



            for i in range(t_d.shape[0]-1):
               mask=(t_c>=t_d[i]) &(t_c<=t_d[i+1])
               t_interval=t_c[mask]
               tau=t_interval-t_d[i]
               h_k=t_d[3]-t_d[2]


               delta_dot_interval=delta_dot_opt[i] + (tau)*(delta_dot_opt[i+1]-delta_dot_opt[i])/h_k
               v_interval=v_opt[i] + (tau)*(v_opt[i+1]-v_opt[i])/h_k
               x_interval=x_opt[i] +  v_opt[i]*np.cos(theta_opt[i])*tau + (tau**2/(2*h_k))*(v_opt[i+1]*np.cos(theta_opt[i+1])-v_opt[i]*np.cos(theta_opt[i]))
               y_interval=y_opt[i] +  v_opt[i]*np.sin(theta_opt[i])*tau + (tau**2/(2*h_k))*(v_opt[i+1]*np.sin(theta_opt[i+1])-v_opt[i]*np.sin(theta_opt[i]))

               theta_interval=theta_opt[i] +  (1/L)*v_opt[i]*np.tan(delta_opt[i])*tau + (tau**2/(2*h_k))*((1/L)*v_opt[i+1]*np.tan(delta_opt[i+1])-(1/L)*v_opt[i]*np.tan(delta_opt[i]))
               delta_interval=delta_opt[i] + delta_dot_opt[i]*tau + (tau**2/(2*h_k))*(delta_dot_opt[i+1]-delta_dot_opt[i])


               delta_dot_opt_c[mask]=delta_dot_interval
               v_opt_c[mask]=v_interval
               x_opt_c[mask]=x_interval
               y_opt_c[mask]=y_interval
               theta_opt_c[mask]=theta_interval
               delta_opt_c[mask]=delta_interval
               return (x_opt_c,y_opt_c,theta_opt_c,delta_opt_c,delta_dot_opt_c,v_opt_c,t_c)


