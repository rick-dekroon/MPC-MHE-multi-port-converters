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
  <b>Figure 1:</b> Comparison of existing AC- and DC-coupled system structures with the proposed multi-port converter system structure.
</p>

Figure 1 illustrates different system structures for integrating photovoltaic (PV) sources, batteries, and the electrical grid in single-phase installations [2].
The AC-coupled system connects multiple AC sources and loads through a common AC bus, while the DC-coupled system connects DC sources and loads through a common DC bus.
The multi-port converter is a power electronic device that can connect multiple energy sources and loads through a single converter architecture.
Since the multi-port converter system structure is a single converter architecture, it uses fewer components than the AC- and DC-coupled system structures.
Fewer components lead to a lower cost, lower losses (higher efficiency), and a smaller footprint.
Additionally, the components of the converters in the AC-coupled and DC-coupled system structures do not experience the same electrical and thermal stress at all times.
While the components of the multi-port converter system structure experience the same electrical and thermal stress at all times, which leads to a more uniform aging of the components, hence a higher realiablity.

The topology of the multi-port converter is already automated in [3].
However, to fully realize the potential of these systems, it is important to also consider the automation of control and estimation strategies.
Automated approaches can help streamline the integration of multi-port converters into diverse applications, reduce manual engineering effort, and facilitate rapid adaptation to evolving system requirements.
This context motivates the need for further research into systematic methods for automating not only the topology, but also the associated control and estimation design for multi-port converter architectures.

## Problem Statement

While multi-port converter architectures offer significant advantages in terms of integration, efficiency, and reliability, their operation introduces new challenges for control and estimation. Existing methods are often tailored to single-port or simpler multi-converter systems and do not fully address the complexity, interactions, and constraints inherent in multi-port topologies. As the number of ports and system requirements increase, manual design of controllers and estimators becomes increasingly time-consuming, error-prone, and difficult to scale.

There is a clear need for systematic and automated approaches to the design of control and estimation strategies that can ensure stability, performance, and robustness for multi-port converter architectures. Addressing this gap is essential for enabling reliable, cost-effective, and scalable deployment of advanced power electronic systems in modern energy networks.

## Research Questions and Objectives

This thesis extends the work of [2, 3] by investigating the control and estimation strategies for multi-port converter architectures.
The research will be guided by the following research questions:

- What different kinds of control and estimation strategies exist for multi-port converter architectures? Which control and estimation strategy is most suitable for automation?
- How can this control and estimation strategy be deployed on multi-port converter architectures?
  - How can a model of the system dynamics be derived?
  - What are the key design steps involved in this control and estimation strategy?
  - What challenges arise during the implementation of this control and estimation strategy?
  - How robust is this control and estimation strategy to system uncertainties?
  - How does this control and estimation strategy perform in simulation studies?
- How can this control and estimation strategy be automated for multi-port converter architectures?
  - How can a model of the system dynamics be automatically derived?
  - What are the difficulties in automating this control and estimation strategy?

## Structure

The thesis is organized as follows. Chapter 1 introduces the motivation, context, problem statement, research questions, and objectives. Chapter 2 provides a literature review on modeling, parameterization, control, and estimation strategies for power electronic converters, including robustness and stability assessment methods. Chapter 3 presents the design and implementation of the controller for multi-port converter architectures, covering model derivation, optimal control problem formulation, terminal conditions, implementation details, robustness analysis, and simulation results. Chapter 4 details the design and integration of the estimator with the controller, including estimator formulation, implementation, robustness to measurement noise, and simulation results. Chapter 5 discusses the automation of controller and estimator design, focusing on automated model derivation, tuning strategies, and practical aspects of real-time implementation. Finally, Chapter 6 summarizes the main findings and provides recommendations for future work.

## References

[1] International Energy Agency. (2024). *Energy Technology Perspectives 2024*. IEA, Paris. [CC BY 4.0 License](https://www.iea.org/reports/energy-technology-perspectives-2024).

[2] Deckers, M., Van Cappellen, L., Emmers, G., Poormohammadi, F., & Driesen, J. (2022). Cost comparison for different PV-battery system architectures including power converter reliability. In *2022 24th European Conference on Power Electronics and Applications (EPE’22 ECCE Europe)* (pp. 1–11). IEEE.

[3] Deckers, M., & Driesen, J. (2024). Automated power converter topology derivation methodology based on exhaustive graph search. *IEEE Transactions on Power Electronics*.
