{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    astra-msgs.url = "github:SHC-ASTRA/astra_msgs/main";
    astra-msgs.inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };
  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
      astra-msgs,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        applyDistroOverlay =
          rosOverlay: rosPackages:
          rosPackages
          // builtins.mapAttrs (
            rosDistro: rosPkgs: if rosPkgs ? overrideScope then rosPkgs.overrideScope rosOverlay else rosPkgs
          ) rosPackages;
        rosDistroOverlays = final: prev: {
          # Apply the overlay to multiple ROS distributions
          rosPackages = applyDistroOverlay (import ./overlay.nix) prev.rosPackages;
        };
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            rosDistroOverlays
          ];
        };

        rosDistro = "humble";

      in
      {
        legacyPackages = pkgs.rosPackages;
        packages = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro};
        checks = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro};
        devShells.default = import ./shell.nix {
          inherit pkgs rosDistro;
          extraPkgs = { };
          extraPaths = [
            astra-msgs.packages.${system}.astra-msgs
          ];
        };
      }
    );
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
