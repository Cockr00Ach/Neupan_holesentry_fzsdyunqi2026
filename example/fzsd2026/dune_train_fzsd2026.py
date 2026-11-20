import os

from neupan import neupan

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(CUR_DIR, 'model', 'fzsd2026_sentry_3d')


def latest_checkpoint(model_dir):
    """Return (path, next_epoch) of the highest-numbered model_*.pth, or (None, 0)."""
    if not os.path.isdir(model_dir):
        return None, 0
    epochs = []
    for name in os.listdir(model_dir):
        if name.startswith('model_') and name.endswith('.pth'):
            try:
                epochs.append((int(name[len('model_'):-len('.pth')]), name))
            except ValueError:
                continue
    if not epochs:
        return None, 0
    epoch, name = max(epochs)
    return os.path.join(model_dir, name), epoch + 1


if __name__ == '__main__':
    neupan_planner = neupan.init_from_yaml('dune_train_fzsd2026.yaml')

    total_epoch = neupan_planner.dune_train_kwargs.get('epoch', 5000)
    checkpoint, start_epoch = latest_checkpoint(MODEL_DIR)

    if checkpoint is not None and start_epoch <= total_epoch:
        print(f'resume from {checkpoint}, start_epoch {start_epoch}/{total_epoch}')
        neupan_planner.dune_train_kwargs['resume_checkpoint'] = checkpoint
        neupan_planner.dune_train_kwargs['start_epoch'] = start_epoch

    neupan_planner.train_dune()
