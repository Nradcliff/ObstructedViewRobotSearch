from transformers import Trainer, TrainingArguments

class LLMProcessor:
    def __init__(self):
        self.tokenizer = AutoTokenizer.from_pretrained("meta-llama/LLaMA-3.2-90b")
        self.model = AutoModelForCausalLM.from_pretrained("meta-llama/LLaMA-3.2-90b")

    def generate_text(self, prompt):
        inputs = self.tokenizer(prompt, return_tensors="pt")
        outputs = self.model.generate(**inputs)
        return self.tokenizer.decode(outputs[0], skip_special_tokens=True)

    def get_response(self, prompt):
        response = self.generate_text(prompt)
        return response

    def get_suggestions(self, prompt, num_suggestions=5):
        suggestions = []
        for i in range(num_suggestions):
            suggestion = self.generate_text(prompt + f" Suggestion {i+1}:")
            suggestions.append(suggestion)
        return suggestions

    def fine_tune(self, train_dataset, eval_dataset, output_dir="./model"):
        training_args = TrainingArguments(
            output_dir=output_dir,
            evaluation_strategy="epoch",
            learning_rate=5e-5,
            per_device_train_batch_size=2,
            per_device_eval_batch_size=2,
            num_train_epochs=3,
            weight_decay=0.01,
        )
        trainer = Trainer(
            model=self.model,
            args=training_args,
            train_dataset=train_dataset,
            eval_dataset=eval_dataset,
        )
        trainer.train()

    def save_model(self, save_directory):
        self.model.save_pretrained(save_directory)
        self.tokenizer.save_pretrained(save_directory)

    def load_model(self, load_directory):
        self.model = AutoModelForCausalLM.from_pretrained(load_directory)
        self.tokenizer = AutoTokenizer.from_pretrained(load_directory)
