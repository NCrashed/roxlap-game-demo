let
  rust_overlay = import (builtins.fetchTarball "https://github.com/oxalica/rust-overlay/archive/master.tar.gz");
  pkgs = import <nixpkgs> { overlays = [ rust_overlay ]; };
  rustToolchain = pkgs.rust-bin.nightly.latest.default.override {
    extensions = [ "rust-src" "rust-analyzer" ];
  };
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    rustToolchain
    SDL2
    SDL2_mixer
    SDL2_gfx
    SDL2_ttf
  ];

  shellHook = ''

  '';
}