"""
rclpy の購読・配信で、既定の QoS イベント (QoS の不一致の通知) を作らない。

★ jetracer_common でただ 1 つ rclpy に依存するモジュール (ROS ノードが明示的に import したときだけ読まれる。
  単体テストや学習側からは読まない)。minicarbattle2026 の minicar_common.rclpy_lean と同じもの (2026-09-28)。

読み込むだけで効く (import jetracer_common.rclpy_lean)。rclpy (Humble) は購読・配信のたびに
「QoS の不一致」の通知を受けるイベントを 1 つずつ作り、起こされるたびに全部を待ちの集合 (wait set) へ
並べ直す。20〜100 Hz のトピックで毎秒数百回起こされるノードでは、この並べ直しが自前の処理より重くなる
(M-05 の sim で測った値: 全ノードで自前の処理 14%、rclpy の待ち・取り出し 78%。イベントを作らないと CPU が約 17% 減った)。
失うのは「QoS が合わない相手がいる」という警告のログだけ (通信そのものは変わらない)。
呼び出し側が event_callbacks を渡したときは、そちらを使う。
"""
import rclpy.node as _node
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks

_create_subscription = _node.Node.create_subscription
_create_publisher = _node.Node.create_publisher


def _lean_create_subscription(self, *args, **kwargs):
    kwargs.setdefault('event_callbacks', SubscriptionEventCallbacks(use_default_callbacks=False))
    return _create_subscription(self, *args, **kwargs)


def _lean_create_publisher(self, *args, **kwargs):
    kwargs.setdefault('event_callbacks', PublisherEventCallbacks(use_default_callbacks=False))
    return _create_publisher(self, *args, **kwargs)


if not getattr(_node.Node, '_jetracer_lean', False):
    _node.Node.create_subscription = _lean_create_subscription
    _node.Node.create_publisher = _lean_create_publisher
    _node.Node._jetracer_lean = True
