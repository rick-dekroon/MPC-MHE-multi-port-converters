# Automated controller and compensator design for multi-port converter architectures

## Motivation and Context

Existing power systems are undergoing a significant transformation, driven by the integration of renewable energy sources, the electrification of transportation, and the increasing demand for energy efficiency [1].
A contribution to this transformation is the development of multi-port converters proposed in [2, 3].

<table align="center" style="width:100%;">
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/da619579-ff3c-45c0-833f-3bcd63b54e7d" alt="Subfigure 1" width="300"><br>
      <p style="text-align: center;">(a) AC-coupled.</p>
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/db88b50a-e0ad-4f26-ba5f-a7ab106995c5" alt="Subfigure 2" width="300"><br>
      <p style="text-align: center;">(b) DC-coupled.</p>
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/e9eb9efc-01c6-454b-b5de-7e91bd1db5c0" alt="Subfigure 3" width="300"><br>
      <p style="text-align: center;">(c) Multi-port converter.</p>
    </td>
  </tr>
</table>
<p align="center" style="width:100%;"">
  <b>Figure 1:</b> Comparison of existing AC- and DC-coupled system structures with the proposed multi-port converter system structure [2].
</p>

Figure 1 illustrates different system structures for integrating photovoltaic (PV) sources, batteries, and the electrical grid in single-phase installation [2].
The AC-coupled system in Figure 1a connects multiple AC sources and loads through a common AC bus, while the DC-coupled system in Figure 1b connects DC sources and loads through a common DC bus.
The multi-port converter in Figure 1c is a power electronic device that can connect multiple energy sources and loads through a single converter architecture.
Since the multi-port converter system structure is a single converter architecture, it uses fewer components than the AC- and DC-coupled system structures.
Fewer components lead to a lower cost, lower losses (higher efficiency), and a smaller footprint.
Additionally, the multi-port converter enables multiple power flows to be combined within the same components, resulting in less fluctuating loading for each component. This leads to more uniform aging and, consequently, higher reliability.

Designing the topology and compensators for multi-port converter architectures demands greater effort, as these solutions must be customized for each specific application.
The topology derivation of the multi-port converter is already automated in [3].
However, to fully realize the potential of these systems, it is important to also consider the automation of control and estimation strategies.
Automated approaches can help streamline the integration of multi-port converters into diverse applications, reduce manual engineering effort, and facilitate rapid adaptation to evolving system requirements.
This context motivates the need for further research into systematic methods for automating not only the topology, but also the associated control and estimation design for multi-port converter architectures.

## Problem Statement

While multi-port converter architectures offer significant advantages in terms of integration, efficiency, and reliability, their operation introduces new challenges for control and estimation. Existing methods are often tailored to single-port or simpler multi-converter systems and do not fully address the complexity, interactions, and constraints inherent in multi-port topologies. As the number of ports and system requirements increase, manual design of controllers and estimators becomes increasingly time-consuming, error-prone, and difficult to scale.

There is a clear need for systematic and automated approaches to the design of control and estimation strategies that can ensure stability, performance, and robustness for multi-port converter architectures. Addressing this gap is essential for enabling reliable, cost-effective, and scalable deployment of advanced power electronic systems in modern energy networks.

## Repository Structure

Model Predictive Control (MPC) and Moving Horizon Estimation (MHE) are advanced techniques implemented for the control and state estimation of multi-port converters. This repository provides a modular framework developed in `MATLAB/Simulink 2024b` and `PLECS 4.8.9 blockset`, enabling rapid prototyping, simulation, and validation of MPC and MHE strategies tailored to multi-port converter architectures.

The three multi-port converter topologies proposed in [3] have been simulated and organized for clarity and ease of use. Under the `simulate` directory, you will find three main folders: `MIMO_Case_A`, `MIMO_Case_B`, and `MIMO_Case_C`, each corresponding to a specific converter topology.

For `MIMO_Case_A`, both the MPC controller and the combined MPC/MHE compensator implementations are included within the `grid-following` and `grid-forming` subfolders. Additionally, this case provides three dedicated MATLAB files—[ellipsoid_invariant_set.m](simulate/MIMO_Case_A/ellipsoid_invariant_set.m), [maximal_controlled_positive_invariant_set.m](simulate/MIMO_Case_A/maximal_controlled_positive_invariant_set.m), and [maximal_positive_invariant_set.m](simulate/MIMO_Case_A/maximal_positive_invariant_set.m)—for visualizing the different terminal conditions of the MPC controller.

For `MIMO_Case_B` and `MIMO_Case_C`, each case folder contains `grid-following` and `grid-forming` subfolders, but only the MPC/MHE compensator implementation is provided for these topologies. This structure enables straightforward comparison and testing of the compensator-based control strategies across all three multi-port converter topologies.

## References

1. International Energy Agency. (2024). *Energy Technology Perspectives 2024*. IEA, Paris. [CC BY 4.0 License](https://www.iea.org/reports/energy-technology-perspectives-2024).

2. Deckers, M., Van Cappellen, L., Emmers, G., Poormohammadi, F., & Driesen, J. (2022). Cost comparison for different PV-battery system architectures including power converter reliability. *Proceedings of the 2022 24th European Conference on Power Electronics and Applications (EPE’22 ECCE Europe)*, IEEE, pp. 1–11.

3. Deckers, M., & Driesen, J. (2024). Automated power converter topology derivation methodology based on exhaustive graph search. *IEEE Transactions on Power Electronics*.
