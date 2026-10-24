# Windows配布版のライセンス確認

確認日: 2026-10-21 / 0.2.0

本体と人工サンプルは独自利用条件（LICENSE）です。本体のライセンスを同梱ライブラリに適用するものではありません。
この配布版はPython・NumPy・SciPy・Tcl/Tkと実行に必要なライブラリを含むため ソース版0.1.0とは同梱物が異なります。
インストールされた配布物の全ライセンスファイルと同梱パッケージ内のライセンス/NOTICEを保持しました。

- Python: PSFライセンス等。Python自体のソース変更はありません。必要な実行時モジュールを同梱。
- NumPy / SciPy: BSD系の主ライセンスに加え OpenBLAS/LAPACK/GCC runtime等の条件を原文で保持。
- GCC runtime: GPL-3.0-or-later WITH GCC-exception-3.1。組み合わせた実行形式に適用される例外条項を確認。
- Tcl/Tk: 著作権・許諾文を保持して再配布。
- OpenSSL: Apache-2.0。原文を保持。
- libffi / cffi / pycparser / charset-normalizer / typing-extensions / setuptools / wheelおよび同梱補助コード: 対応する原文とNOTICEを参照。
- packaging: Apache-2.0またはBSD系の許諾。両方の原文を保持。
- PyInstaller: GPL例外により 生成した実行形式のライセンスを選べることを確認。PyInstaller自体は改変していません。

Microsoftのランタイムについては Python-Windows-LICENSE.txt の Additional Conditions for this Windows binary build を含めて保持しています。
Microsoft部分の著作権・商標・特許表示を改変せず Windows以外への配布やMicrosoftの推奨を装う表示 悪意・欺瞞・違法な用途への組込みをしないでください。
これらの条件はMicrosoft部分に適用され 本体の独自利用条件を置き換えるものではありません。

公式の確認先:
https://pyinstaller.org/en/stable/license.html
https://docs.python.org/3/license.html
https://numpy.org/doc/stable/license.html
https://github.com/scipy/scipy/blob/main/LICENSE.txt
https://gcc.gnu.org/onlinedocs/libstdc++/manual/license.html
https://github.com/tcltk/tcl/tree/core-8-6-12
https://github.com/openssl/openssl/tree/openssl-3.5.8
https://github.com/libffi/libffi

外部データの公開権や元案件の契約を確認するものではありません。制作素材・実収録モーションはこの配布物に含めていません。
