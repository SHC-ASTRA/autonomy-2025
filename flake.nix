{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    astra-msgs.url = "github:SHC-ASTRA/astra_msgs/main";
    astra-msgs.inputs.nix-ros-overlay.follows = "nix-ros-overlay";

    treefmt = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };
  outputs = { self, nix-ros-overlay, nixpkgs, astra-msgs, ... }@inputs:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        applyDistroOverlay = rosOverlay: rosPackages:
          rosPackages // builtins.mapAttrs (rosDistro: rosPkgs:
            if rosPkgs ? overrideScope then
              rosPkgs.overrideScope rosOverlay
            else
              rosPkgs) rosPackages;
        rosDistroOverlays = final: prev: {
          # Overlay opencv4 to build it with GTK2 for GUI
          opencv4 = prev.opencv4.override { enableGtk2 = true; };

          # Apply the overlay to multiple ROS distributions
          rosPackages =
            applyDistroOverlay (import ./overlay.nix) prev.rosPackages;
        };
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default rosDistroOverlays ];
        };
        astra_msgs_pkgs = astra-msgs.packages.${system};

        rosDistro = "humble";

      in {
        legacyPackages = pkgs.rosPackages;
        packages = builtins.intersectAttrs (import ./overlay.nix null null)
          pkgs.rosPackages.${rosDistro};
        checks = builtins.intersectAttrs (import ./overlay.nix null null)
          pkgs.rosPackages.${rosDistro};
        devShells.default = import ./shell.nix {
          inherit pkgs rosDistro;
          extraPkgs = { };
          extraPaths = [ astra_msgs_pkgs.astra-msgs ];
        };

        formatter = (inputs.treefmt.lib.evalModule pkgs
          ./treefmt.nix).config.build.wrapper;

      });
  nixConfig = {
    extra-substituters =
      [ "https://ros.cachix.org" "https://attic.iid.ciirc.cvut.cz/ros" ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      "ros:JR95vUYsShSqfA1VTYoFt1Nz6uXasm5QrcOsGry9f6Q="
    ];
  };
}
