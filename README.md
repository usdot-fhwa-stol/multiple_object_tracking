# Multiple object tracking

This repository holds a multiple object tracking (MOT) library to support cooperative perception. This library is intended for tracking object level data from multiple sources where the input is abstracted as a standard cooperative perception object tracking interface. For example, object level data can come from multiple sources including J2334 Sensor Data Sharing Message, Basic Safety Message, or local perception. This enables the library to be deployed in different road actors such as C-ADS equipped vehicles or the infrastructure with the help of message adapters that are suited for respective middleware.  

The library exposes multiple submodules and functionalities adopted from the architectural and algorithmic advancements made by the sensor fusion community. Example usage of this library to fully execute multiple object tracking pipeline is implemented in CARMA Platform [here]([url](https://github.com/usdot-fhwa-stol/carma-platform/tree/develop/carma_cooperative_perception)). More documentation for implementation detail can be found in the `docs` folder.

| CI Build Status | Sonar Code Quality |
|----------------------|---------------------|
|[![CI](https://github.com/usdot-fhwa-stol/multiple_object_tracking/actions/workflows/ci.yml/badge.svg)](https://github.com/usdot-fhwa-stol/multiple_object_tracking/actions/workflows/ci.yml) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=usdot-fhwa-stol_multiple-object-tracking&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=usdot-fhwa-stol_multiple-object-tracking) |
## Contribution
Welcome to the CARMA contributing guide. Please read this guide to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project. [CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Contributing.md)

## Code of Conduct
Please read our [CARMA Code of Conduct](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Code_of_Conduct.md) which outlines our expectations for participants within the CARMA community, as well as steps to reporting unacceptable behavior. We are committed to providing a welcoming and inspiring community for all and expect our code of conduct to be honored. Anyone who violates this code of conduct may be banned from the community.

## Attribution
The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [Project Attribution](<ATTRIBUTION.md>)

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [Project License](<docs/License.md>)

## Contact
Please click on the CARMA logo below to visit the Federal Highway Administration (FHWA) CARMA website.

[![CARMA Image](https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/docs/image/CARMA_icon.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)
