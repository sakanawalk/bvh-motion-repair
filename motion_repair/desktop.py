"""Japanese desktop application. Processing is local and never overwrites input."""
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import traceback
import uuid
import webbrowser

from .cli import run, validate, PROFILE
from .engine import BVH

def resources():
    return Path(getattr(sys,'_MEIPASS',Path(__file__).resolve().parents[1]))

def friendly_error(error):
    message=str(error)
    translations=[
        ('120 fps','このファイルは約120fpsではありません。書き出し元で120fpsのBVHを作成してください。'),
        ('120 frames','モーションが短すぎます。1秒以上のBVHを選んでください。'),
        ('Missing required bones','骨の名前が対応形式と異なります。使い方の対応骨格とBVHの骨名を確認してください。'),
        ('Root must','骨格の一番上の骨が対応形式と異なります。BVHの書き出し設定を確認してください。'),
        ('channels','骨の回転・移動の記録形式が対応していません。書き出し設定と対応プロファイルを確認してください。'),
        ('hierarchy','骨のつながり方が対応形式と異なります。書き出し設定と対応プロファイルを確認してください。'),
        ('Arms must','腕の軸方向が対応形式と異なります。このファイルの自動修正には対応していません。'),
        ('centimeter','骨格の大きさが対応範囲外です。センチメートル単位で書き出したBVHか確認してください。'),
        ('already exists','同じ保存先がすでにあります。もう一度実行すると新しい保存先を作ります。'),
        ('Source changed','処理中に元ファイルが変更されました。編集や同期を止めてからやり直してください。'),
        ('Nonfinite','ファイルに無効な数値が含まれています。BVHを書き出し直してください。'),
        ('Duplicate','同じ名前の骨が重複しています。この骨格形式には対応していません。'),
        ('Degenerate','長さがほぼゼロの骨が含まれています。この骨格形式には対応していません。')]
    for key,text in translations:
        if key.lower() in message.lower():return text
    if isinstance(error,PermissionError):return 'ファイルにアクセスできません。書き込み可能な保存先を選び、ファイルが別のアプリで開かれていないか確認してください。'
    if isinstance(error,FileNotFoundError):return 'ファイルが見つかりません。「BVHを選ぶ」から選び直してください。'
    if isinstance(error,MemoryError):return 'メモリが足りません。他の大きなアプリを閉じてからやり直してください。'
    return 'このデータの読み込み、または修正結果の検証に失敗しました。元ファイルは上書きしていません。配布ページの問い合わせ先へ、個人パスを除いたログと再現手順を送ってください。'

class App:
    def __init__(self,root,results=None):
        self.root=root;self.events=queue.Queue();self.busy=False;self.result=None;self.validated=None
        self.output_parent=Path(results) if results else Path.home()/'Documents'/'モーション修正結果'
        self.source=tk.StringVar(value='まだファイルを選んでいません')
        self.status=tk.StringVar(value='まずはBVHファイルを選んでください')
        self.details=tk.StringVar(value='対応プロファイルの約120fpsの人型BVH向けです。')
        self.destination=tk.StringVar(value=str(self.output_parent))
        self.confirmed=tk.BooleanVar(value=False)
        self.elapsed=tk.StringVar(value='')
        root.title('モーション修正ツール | 0.2.0 r5');root.geometry(f'860x{min(760,max(600,root.winfo_screenheight()-110))}');root.minsize(800,600)
        root.configure(bg='#f4f6fa')
        style=ttk.Style();style.theme_use('clam')
        style.configure('.',font=('Yu Gothic UI',11))
        style.configure('TFrame',background='#f4f6fa');style.configure('TLabel',background='#f4f6fa',foreground='#223047')
        style.configure('Title.TLabel',font=('Yu Gothic UI',24,'bold'))
        style.configure('Section.TLabel',font=('Yu Gothic UI',13,'bold'))
        style.configure('TButton',padding=(14,9));style.configure('Primary.TButton',background='#2457c5',foreground='white',font=('Yu Gothic UI',12,'bold'))
        style.map('Primary.TButton',background=[('disabled','#b8c6dd'),('active','#16419d')])
        style.configure('TCheckbutton',background='#f4f6fa',font=('Yu Gothic UI',10))
        viewport=tk.Canvas(root,background='#f4f6fa',highlightthickness=0)
        scrollbar=ttk.Scrollbar(root,orient='vertical',command=viewport.yview)
        scrollbar.pack(side='right',fill='y');viewport.pack(side='left',fill='both',expand=True)
        viewport.configure(yscrollcommand=scrollbar.set)
        outer=ttk.Frame(viewport,padding=24)
        viewport_window=viewport.create_window((0,0),window=outer,anchor='nw')
        outer.bind('<Configure>',lambda e:viewport.configure(scrollregion=viewport.bbox('all')))
        viewport.bind('<Configure>',lambda e:viewport.itemconfigure(viewport_window,width=e.width))
        root.bind_all('<MouseWheel>',lambda e:viewport.yview_scroll(-int(e.delta/120),'units'))
        ttk.Label(outer,text='モーションの飛び・ねじれを整える',style='Title.TLabel').pack(anchor='w')
        ttk.Label(outer,text='元のファイルを残して、修正候補と比較画面を作ります。',padding=(0,5,0,16)).pack(anchor='w')
        ttk.Label(outer,text='1  モーションを選ぶ',style='Section.TLabel').pack(anchor='w')
        row=ttk.Frame(outer);row.pack(fill='x',pady=(9,6))
        self.choose=ttk.Button(row,text='BVHを選ぶ',command=self.pick);self.choose.pack(side='left')
        self.demo=ttk.Button(row,text='お試しデータを使う',command=self.pick_demo);self.demo.pack(side='left',padx=8)
        self.file_label=ttk.Label(outer,textvariable=self.source,wraplength=790);self.file_label.pack(anchor='w',pady=(0,6))
        ttk.Label(outer,textvariable=self.details,wraplength=790,foreground='#52627b').pack(anchor='w')
        ttk.Separator(outer).pack(fill='x',pady=17)
        ttk.Label(outer,text='2  保存先を確認する',style='Section.TLabel').pack(anchor='w')
        row=ttk.Frame(outer);row.pack(fill='x',pady=8)
        self.output_button=ttk.Button(row,text='保存先を変更',command=self.pick_output);self.output_button.pack(side='right')
        ttk.Label(row,textvariable=self.destination,wraplength=600).pack(side='left')
        ttk.Label(outer,text='この中に、実行するたびに新しいフォルダを作ります。',foreground='#52627b').pack(anchor='w')
        ttk.Separator(outer).pack(fill='x',pady=17)
        ttk.Label(outer,text='3  自動修正を実行する',style='Section.TLabel').pack(anchor='w')
        self.check=ttk.Checkbutton(outer,text='対応プロファイルの形式（120fps・Yが上・cm単位）で書き出したBVHです',variable=self.confirmed,command=self.enable)
        self.check.pack(anchor='w',pady=(8,8))
        ttk.Label(outer,text='自動修正は候補です。完成後に比較を見て、意図した動きが残っているか確認してください。',wraplength=790,foreground='#52627b').pack(anchor='w')
        self.start=ttk.Button(outer,text='修正する',style='Primary.TButton',command=self.process,state='disabled');self.start.pack(anchor='w',pady=(13,9))
        self.bar=ttk.Progressbar(outer,mode='indeterminate');self.bar.pack(fill='x')
        ttk.Label(outer,textvariable=self.status,wraplength=790,style='Section.TLabel').pack(anchor='w',pady=(9,2))
        ttk.Label(outer,textvariable=self.elapsed,foreground='#52627b').pack(anchor='w')
        row=ttk.Frame(outer);row.pack(fill='x',pady=(10,0))
        self.review=ttk.Button(row,text='修正前後を比較する',command=self.open_review,state='disabled');self.review.pack(side='left')
        self.folder=ttk.Button(row,text='結果フォルダを開く',command=self.open_folder,state='disabled');self.folder.pack(side='left',padx=8)
        ttk.Button(row,text='使い方',command=self.help).pack(side='right')
        root.protocol('WM_DELETE_WINDOW',self.close)
        root.after(100,self.poll)

    def enable(self):
        self.start.configure(state='normal' if self.validated and self.confirmed.get() and not self.busy else 'disabled')

    def set_busy(self,value):
        self.busy=value
        for widget in [self.choose,self.demo,self.output_button,self.check]:widget.configure(state='disabled' if value else 'normal')
        self.enable()
        if value:self.started=time.monotonic();self.bar.start(12)
        else:self.bar.stop()

    def pick(self):
        path=filedialog.askopenfilename(title='修正したいBVHを選んでください',filetypes=[('BVHモーション','*.bvh')])
        if path:self.load(Path(path))

    def pick_demo(self):
        self.load(resources()/'sample'/'sample.bvh',demo=True)

    def load(self,path,demo=False):
        self.validated=None;self.result=None;self.confirmed.set(demo);self.source.set('お試しモーション（4秒）' if demo else path.name)
        self.review.configure(state='disabled');self.folder.configure(state='disabled')
        self.status.set('対応形式か確認しています');self.details.set('確認中…');self.set_busy(True)
        def task():
            try:
                b=BVH(path);validate(b)
                self.events.put(('loaded',(path,b.frame_count,b.dt,demo)))
            except Exception as error:self.events.put(('error',(error,traceback.format_exc())))
        threading.Thread(target=task,daemon=True).start()

    def pick_output(self):
        path=filedialog.askdirectory(title='結果を保存する場所を選んでください')
        if path:self.output_parent=Path(path);self.destination.set(path)

    def process(self):
        if not self.validated or not self.confirmed.get() or self.busy:return
        source=self.validated
        out=self.output_parent/(source.stem+'_修正_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        self.result=None;self.review.configure(state='disabled');self.folder.configure(state='disabled')
        self.status.set('修正を開始しています');self.set_busy(True)
        def task():
            try:
                result=run(source,out,PROFILE,progress=lambda msg:self.events.put(('progress',msg)))
                self.events.put(('done',result))
            except Exception as error:self.events.put(('error',(error,traceback.format_exc())))
        threading.Thread(target=task,daemon=True).start()

    def poll(self):
        if self.busy:self.elapsed.set('経過 '+str(int(time.monotonic()-self.started))+' 秒 ｜ 処理中もこの画面は操作できます')
        try:
            while True:
                kind,value=self.events.get_nowait()
                if kind=='progress':self.status.set(value)
                elif kind=='loaded':
                    path,n,dt,demo=value;self.validated=path;self.set_busy(False)
                    self.details.set(f'{n:,} フレーム ／ {(n-1)*dt:.1f} 秒 ／ {1/dt:.2f} fps　対応形式の確認OK')
                    self.status.set('お試しデータの準備ができました' if demo else '準備ができました。書き出し形式を確認して「修正する」へ')
                    self.elapsed.set('')
                elif kind=='done':
                    self.result=value;self.set_busy(False);self.review.configure(state='normal');self.folder.configure(state='normal')
                    self.status.set('修正候補ができました。比較して確認してください')
                    self.elapsed.set('採用するファイル：repaired.bvh　／　元ファイルはそのままです')
                elif kind=='error':
                    error,detail=value;self.set_busy(False);self.status.set('処理を完了できませんでした');self.elapsed.set('元ファイルは上書きしていません')
                    messagebox.showerror('ファイルを確認してください',friendly_error(error)+'\n\n詳細：'+str(error))
        except queue.Empty:pass
        self.root.after(120,self.poll)

    def open_review(self):
        if self.result:webbrowser.open((self.result/'review.html').as_uri())
    def open_folder(self):
        if self.result:os.startfile(str(self.result))
    def help(self):
        webbrowser.open((resources()/'guide.html').as_uri())
    def close(self):
        if self.busy:
            messagebox.showinfo('処理中です','処理が終わってから閉じてください。数分かかることがあります。')
        else:self.root.destroy()

def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--results',type=Path);parser.add_argument('--self-test',type=Path)
    args=parser.parse_args()
    if args.self_test:
        # Packaged smoke test: validates GUI runtime and completes the real engine path.
        destination=args.self_test
        root=tk.Tk();root.withdraw();root.update();root.destroy()
        result=run(resources()/'sample'/'sample.bvh',destination,PROFILE)
        (result/'desktop_test.json').write_text(json.dumps({'passed':True,'frozen':bool(getattr(sys,'frozen',False)),'tk':'8.6','external_python_required':False}),encoding='utf-8')
        return
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError,OSError):pass
    root=tk.Tk();App(root,args.results);root.mainloop()

if __name__=='__main__':main()
