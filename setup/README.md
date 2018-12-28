# Setting up the WolvMarine

## Notes

If zsh is used instead of bash, be sure to still modify the computer's `.bashrc`
file so that bash scripts execute as intended.

## Action items

- `cRIO/` is a placeholder for setting up a server for the cRIO. This may not
  be used at all. The labview files do need to be deposited here after being
  tested. Ideally, a server interface (albeit a simple one) should be set up
  on the cRIO such that it won't have to constantly send data, and so that
  frequency ranges can be specified by ROS rather than being statically
  determined by the cRIO's loop.

- `network/` needs to be populated with a well documented and scripted set up
  procedure for the WolvMarine.

- `software/` needs to be updated and possibly overhauled.
