from arm_1.model.kerasmodel import KerasModel

model = KerasModel('model_64x64_b4_e2000_p2')
model.build_pipeline()
model.train()
model.save()


