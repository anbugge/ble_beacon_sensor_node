## Build Instructions

* Install Simplicity Studio v5 with Bluetooth support. Tested on Simplicity Studio 5.1 with Gecko SDK 3.0.
* Create an soc_empty example project for your board (BRD4166A or BRD4184A)
* Close Studio
* Delete the automatically generated `app.*` and `*.slcp` files in the demo project
* Add the relevant files from this project (symlinking works fine)
* Re-open Studio
* Open the `.slcp` file in the Studio project and click Force Generation
* With some luck it will compile and run

## Tokens
The app is configured using custom tokens. Edit `tokens.txt` as required, and flash using Simplicity Commander:

`commander flash --tokendefs tokens.json --tokenfile tokens.txt`

`tokens.json` contains the definitions itself - new tokens can be added here. To update the token header used by the app run

`commander tokenheader --tokendefs tokens.json brd4184a/tokens.h`