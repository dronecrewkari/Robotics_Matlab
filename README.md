# Robotics_Matlab
The technology of robotics programed by Matlab code
$$
\begin{aligned} \overline{\mu}_{t} &=g\left(u_{t}, \mu_{t-1}\right) \\ \overline{\Sigma}_{t} &=G_{t} \Sigma_{t-1} G_{t}^{T}+R_{t} \\ K_{t} &=\overline{\Sigma}_{t} H_{t}^{T}\left(H_{t} \overline{\Sigma}_{t} H_{t}^{T}+Q_{t}\right)^{-1} \\ \mu_{t} &=\overline{\mu}_{t}+K_{t}\left(z_{t}-h\left(\overline{\mu}_{t}\right)\right) \\ \Sigma_{t} &=\left(I-K_{t} H_{t}\right) \overline{\Sigma}_{t} \end{aligned}
$$
