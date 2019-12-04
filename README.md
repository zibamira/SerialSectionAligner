# Semi-automatic Stitching of Serial Section Image Stacks with Filamentous Structures -- Supplementary Software

This repo contains the source code of the supplementary research software for
the publication

    Norbert Lindow, Florian N. Brünig, Vincent J. Dercksen, Gunar Fabig, Robert
    Kiewisz, Stefanie Redemann, Thomas Müller-Reichert, Steffen Prohaska.
    Semi-automatic Stitching of Serial Section Image Stacks with Filamentous
    Structures. ZIB Report 19-30, 2019.

The software was developed at Zuse Institute Berlin (ZIB).  It is provided
under an MIT-like license.  See details in [LICENSE](./LICENSE.txt).

Binary versions are available at
<https://www.zib.de/software/serial-section-aligner>.

To compile the source code, install Amira with the XPand extension.

In Amira:

 - Use XPand / Development Wizard / Set local Amira directory to set the Amira
   XPand workspace to the toplevel directory of this repo.
 - Use XPand / Development Wizard / Create build system to generate GNU
   Makefiles or Visual Studio project files.

On Linux, prepare the workspace with:

```bash
tar -xvf SerialSectionAligner-dev-lib-LinuxAMD64.tar.bz2
```

Compile with:

```bash
MAKE_CFG=Optimize make
```

On Windows, prepare the workspace with:

```bash
tar -xvf SerialSectionAligner-dev-bin-Win64VC12.tar.bz2
```

Compile in Visual Studio.
