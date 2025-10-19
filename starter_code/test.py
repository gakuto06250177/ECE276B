import os

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

import main

tests = [
    ('single_cube', main.test_single_cube),
    ('maze', main.test_maze),
    ('flappy_bird', main.test_flappy_bird),
    ('window', main.test_window),
    ('tower', main.test_tower),
    ('room', main.test_room),
    ('pillars', main.test_pillars)
]

print('テスト結果サマリー:')
print('='*40)
print(f'実行ディレクトリ: {os.getcwd()}')
print('='*40)

results = []
for name, test_func in tests:
    try:
        success, path_length = test_func(verbose=False)
        status = "✅ 成功" if success else "❌ 失敗"
        results.append((name, status, path_length))
        print(f'{name:12} : {status} (パス長: {path_length:.2f})')
    except Exception as e:
        results.append((name, "⚠️  エラー", 0))
        print(f'{name:12} : ⚠️  エラー')
        print(f'  エラー内容: {str(e)}')

print('='*40)
success_count = sum(1 for _, status, _ in results if "成功" in status)
print(f'成功: {success_count}/{len(tests)} テスト')