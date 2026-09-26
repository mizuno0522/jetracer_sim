#!/bin/bash
# Jetson に仮想ディスプレイ (VNC) を立て、Mac などから見る。物理モニターとは別の画面なので、モニターが
# ちらついても外しても影響しない。デスクトップは軽い xfce (GNOME は仮想ディスプレイだと GL で不安定)。
#
#   sudo ./scripts/setup_vnc.sh            # 導入と自動起動の設定 (1 回だけ)。最後にパスワードを聞く
#   sudo ./scripts/setup_vnc.sh --lan      # SSH トンネル無しで LAN から直接つなぐ (パスワードは暗号化されない)
#   sudo ./scripts/setup_vnc.sh --mirror   # 仮想画面ではなく、モニターに映っている普段のデスクトップ (:0) を共有する (x11vnc・port 5902)
#                                           X の画面データを直接読むので、モニターへの信号が途切れても影響しない
#   ./scripts/setup_vnc.sh status          # 動いているか
#
# 既定 (localhost のみ待ち受け) の Mac からのつなぎ方:
#   Mac のターミナルで   ssh -N -L 5901:localhost:5901 jetson@ubuntu.local      (開いたままにする)
#   VNC Viewer か「画面共有」で   localhost:5901      (画面共有なら Finder → 移動 → サーバへ接続 → vnc://localhost:5901)
# --lan のとき:  VNC Viewer で ubuntu.local:5901 (または Jetson の IP:5901)
#
# 注意: 仮想ディスプレイの中の OpenGL (rviz など) はソフトウェア描画になり遅い。sim の Unity は PC 側なので関係ない。
set -euo pipefail
DISP=1
PORT=$((5900 + DISP))
HERE="$(cd "$(dirname "$0")" && pwd)"

if [ "${1:-}" = status ]; then
  echo "仮想画面 (5901): $(systemctl --user is-active jetracer-vnc.service 2>/dev/null)"
  echo "普段の画面 (5902): $(systemctl --user is-active jetracer-x11vnc.service 2>/dev/null)"
  ss -ltn | grep -E ":590[12] " || echo "5901/5902 とも待ち受けていない"
  exit 0
fi

LAN=0
[ "${1:-}" = --lan ] && LAN=1
[ "$(id -u)" = 0 ] || { echo "sudo で実行すること (導入に apt が要る)" >&2; exit 2; }
U="${SUDO_USER:?sudo から実行すること}"
UH=$(getent passwd "$U" | cut -d: -f6)
UID_N=$(id -u "$U")

if [ "${1:-}" = --mirror ]; then
  MPORT=5902
  echo "== apt: x11vnc"
  apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq x11vnc >/dev/null
  install -d -o "$U" -g "$U" -m 700 "$UH/.vnc"
  if [ ! -s "$UH/.vnc/passwd" ]; then
    echo "== VNC のパスワードを決める"
    sudo -u "$U" x11vnc -storepasswd "$UH/.vnc/passwd"
  fi
  install -d -o "$U" -g "$U" "$UH/.config/systemd/user"
  cat > "$UH/.config/systemd/user/jetracer-x11vnc.service" <<SVC
[Unit]
Description=JetRacer VNC mirror of the desktop :0 (port $MPORT)
After=graphical-session.target

[Service]
Environment=DISPLAY=:0
# -localhost: Jetson の中だけで待ち受け (Mac からは SSH トンネル)。-forever -shared: 切っても終わらない・複数可
ExecStart=/usr/bin/x11vnc -display :0 -auth guess -localhost -rfbport $MPORT -rfbauth %h/.vnc/passwd -forever -shared -noxdamage -quiet
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
SVC
  chown -R "$U:$U" "$UH/.config/systemd"
  sudo -u "$U" XDG_RUNTIME_DIR=/run/user/$UID_N systemctl --user daemon-reload
  sudo -u "$U" XDG_RUNTIME_DIR=/run/user/$UID_N systemctl --user enable --now jetracer-x11vnc.service
  sleep 3
  if ss -ltn | grep -q ":$MPORT "; then
    echo "== 起動した (普段のデスクトップ :0 を port $MPORT で共有)"
    echo "   Mac のターミナル:  ssh -N -L $MPORT:localhost:$MPORT $U@ubuntu.local   (開いたままにする)"
    echo "   Mac の VNC Viewer:  localhost:$MPORT    (画面共有なら vnc://localhost:$MPORT)"
  else
    echo "★ 待ち受けていない。ログ:  journalctl --user -u jetracer-x11vnc -n 30" >&2; exit 1
  fi
  exit 0
fi

echo "== apt: tigervnc と xfce"
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq tigervnc-standalone-server tigervnc-common \
    xfce4 xfce4-terminal dbus-x11 >/dev/null

echo "== $UH/.vnc (xstartup・起動スクリプト)"
install -d -o "$U" -g "$U" -m 700 "$UH/.vnc"
cat > "$UH/.vnc/xstartup" <<'XS'
#!/bin/sh
# 仮想ディスプレイのデスクトップ (xfce)。物理画面の GNOME とは別のセッション
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XDG_SESSION_TYPE=x11
export XDG_CURRENT_DESKTOP=XFCE
exec dbus-launch --exit-with-session startxfce4
XS
chmod 755 "$UH/.vnc/xstartup"
LOCAL=$([ $LAN = 1 ] && echo no || echo yes)
cat > "$UH/.vnc/jetracer-vnc.sh" <<RUN
#!/bin/sh
# systemd (ユーザー) から起動する。前の実行の残り (ロックファイル) を片付けてから Xvnc を前面で動かす
tigervncserver -kill :$DISP >/dev/null 2>&1 || true
rm -f /tmp/.X$DISP-lock /tmp/.X11-unix/X$DISP
exec tigervncserver :$DISP -fg -xstartup "\$HOME/.vnc/xstartup" -geometry 1920x1080 -depth 24 \\
     -localhost $LOCAL -SecurityTypes VncAuth -alwaysshared
RUN
chmod 755 "$UH/.vnc/jetracer-vnc.sh"
chown -R "$U:$U" "$UH/.vnc"

echo "== systemd (ユーザー) の自動起動"
install -d -o "$U" -g "$U" "$UH/.config/systemd/user"
cat > "$UH/.config/systemd/user/jetracer-vnc.service" <<SVC
[Unit]
Description=JetRacer VNC virtual display :$DISP (port $PORT)
After=default.target

[Service]
ExecStart=%h/.vnc/jetracer-vnc.sh
ExecStop=/usr/bin/tigervncserver -kill :$DISP
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
SVC
chown -R "$U:$U" "$UH/.config/systemd"
# ログインしていなくても起動時に立ち上げる
loginctl enable-linger "$U"

if [ ! -s "$UH/.vnc/passwd" ]; then
  echo "== VNC のパスワードを決める (6〜8 文字。閲覧専用パスワードは n でよい)"
  sudo -u "$U" tigervncpasswd "$UH/.vnc/passwd"
fi

sudo -u "$U" XDG_RUNTIME_DIR=/run/user/$UID_N systemctl --user daemon-reload
sudo -u "$U" XDG_RUNTIME_DIR=/run/user/$UID_N systemctl --user enable --now jetracer-vnc.service
sleep 3
if ss -ltn | grep -q ":$PORT "; then
  echo "== 起動した: 待ち受け $(ss -ltn | awk -v p=":$PORT" '$4 ~ p {print $4}' | tr '\n' ' ')"
else
  echo "★ 待ち受けていない。ログ:  journalctl --user -u jetracer-vnc -n 30   /  cat ~/.vnc/*.log" >&2
  exit 1
fi
if [ $LAN = 1 ]; then
  echo "   Mac: VNC Viewer で ubuntu.local:$PORT  (IP なら $(hostname -I | awk '{print $1}'):$PORT)"
else
  echo "   Mac のターミナル:  ssh -N -L $PORT:localhost:$PORT $U@ubuntu.local   (開いたままにする)"
  echo "   Mac の VNC Viewer:  localhost:$PORT    (画面共有なら vnc://localhost:$PORT)"
fi
