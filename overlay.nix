final: prev:
{
  actions-cpp = final.callPackage ././auto_ws/src/actions_cpp/package.nix {};
  clucky-control = final.callPackage ././auto_ws/src/clucky_control/package.nix {};
  clucky-description = final.callPackage ././auto_ws/src/clucky_description/package.nix {};
  macula-pkg = final.callPackage ././auto_ws/src/macula_pkg/package.nix {};
}
