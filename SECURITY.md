# Security Policy

## Supported versions

Security fixes go into `main` and the latest release.

| Version | Supported |
|---|---|
| latest release, `main` | yes |
| older releases | no |

## Reporting a vulnerability

Please **don't open a public issue**. Report it privately through GitHub:
**Security** tab → **Report a vulnerability** ([direct link](https://github.com/kouya-marino/RGB-LiDAR-Fusion/security/advisories/new)).

Include what is affected, how to reproduce it, and its impact. You'll get a reply within a week, and a fix or mitigation will be coordinated with you before details are made public.

## Scope notes

- The web UI (port 8080) has **no authentication** and is meant for a trusted local network or the robot's own network. Don't expose it to the internet.
- The Docker container runs with `--privileged` and host networking so it can reach the sensors; run it only on machines you control.
- Vendored code in `third_party/` (Livox SDK 2 and driver, FAST-LIO2, Three.js) is maintained upstream; issues there are best reported upstream as well.
