# Security and safety reporting

## Supported versions

Until the first public release, only the latest commit on `main` is maintained. After release, supported versions
will be listed here explicitly.

## Reporting

Please do not open a public issue for a vulnerability that could lead to unintended robot motion, collision, or
loss of command arbitration. Use GitHub's private vulnerability reporting for this repository, or contact
Masaaki Hijikata at `hijikata@react-robot.com`. Include the affected revision, configuration, input conditions,
observed output, and a minimal reproducer when possible.

## Safety scope

BAC is navigation software, not a certified safety component. A successful simulation or software test does not
establish that a physical robot can stop safely. Deployment owners remain responsible for sensor coverage,
timeouts, braking characterization, downstream command enforcement, independent protective layers, and validation
on the target robot.
