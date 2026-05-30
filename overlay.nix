final: prev: {
  clucky-detection = final.callPackage ././clucky_detection/package.nix { };
  clucky-localization = final.callPackage ././clucky_localization/package.nix { };
  clucky-nav2 = final.callPackage ././install/clucky_nav2/share/clucky_nav2/package.nix { };
}
